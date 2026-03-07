#!/usr/bin/env python3
import rospy
import psutil
import json
from std_msgs.msg import String

tracked_procs  = {}
pids_scan_counter = 0   # สแกน pid ใหม่ทุก 5 วิ ไม่ใช่ทุกวิ
ai_stats_cache = {}     # รับจาก topic แทนการ scan process

CATEGORIES = [
    ('gmapping',    ['slam_gmapping', 'gmapping']),
    ('move_base',   ['move_base']),
    ('amcl',        ['amcl']),
    ('map_server',  ['map_server']),
    ('ydlidar',     ['ydlidar']),
    ('rosbridge',   ['rosbridge']),
    ('tf2_web',     ['tf2_web_republisher']),
    ('ffmpeg',      ['ffmpeg']),
    ('mediamtx',    ['mediamtx', 'rtsp-simple-server']),
    ('rosserial',   ['rosserial']),
    ('tailscale',   ['tailscale', 'tailscaled']),
    ('stream_mgr',  ['stream_manager']),  #เพิ่ม stream_manager
]

def categorize_process(name, cmdline):
    for cat, keywords in CATEGORIES:
        if any(k in name or k in cmdline for k in keywords):
            return cat
    if '__name:=' in cmdline:
        return 'ros_nodes'
    return 'others'

def ai_stats_callback(msg):
    global ai_stats_cache
    try:
        ai_stats_cache = json.loads(msg.data)
    except Exception:
        pass

def read_temperature():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            return float(f.read()) / 1000.0
    except FileNotFoundError:
        return 0.0

def scan_new_pids():
    """สแกนหา process ใหม่ — เรียกแค่ทุก 5 วิ"""
    for pid in psutil.pids():
        if pid in tracked_procs:
            continue
        try:
            p = psutil.Process(pid)
            name    = p.name().lower()
            cmdline = " ".join(p.cmdline()).lower()
            cat = categorize_process(name, cmdline)
            if cat != 'others':
                p.cpu_percent(interval=None)  # prime ครั้งแรก
                tracked_procs[pid] = {'proc': p, 'cat': cat}
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

def collect_cpu_usage(core_count):
    """อ่าน CPU จาก process ที่ track ไว้แล้ว — เร็วมาก"""
    usage = {cat: 0.0 for cat, _ in CATEGORIES}
    usage['ros_nodes'] = 0.0
    dead_pids = []

    for pid, info in tracked_procs.items():
        try:
            cpu = info['proc'].cpu_percent(interval=None) / core_count
            cat = info['cat']
            if cat in usage:
                usage[cat] += cpu
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            dead_pids.append(pid)

    for pid in dead_pids:
        del tracked_procs[pid]

    return usage

def pi_system_monitor_and_profiler():
    global pids_scan_counter

    rospy.init_node('pi_system_monitor', anonymous=True)
    pub = rospy.Publisher('/pi/system_profile', String, queue_size=5)
    rospy.Subscriber('/stream_manager/ai_stats', String, ai_stats_callback)  # ✅ รับ AI stats

    core_count = psutil.cpu_count() or 4
    rate = rospy.Rate(1)

    rospy.loginfo("Pi Monitor Started (1 Hz) -> /pi/system_profile")
    psutil.cpu_percent(interval=None)  # prime ครั้งแรก
    scan_new_pids()                    # scan ครั้งแรก

    while not rospy.is_shutdown():
        try:
            # สแกน process ใหม่ทุก 5 วิ แทนที่จะทุกวิ
            pids_scan_counter += 1
            if pids_scan_counter >= 5:
                pids_scan_counter = 0
                scan_new_pids()

            total_cpu  = psutil.cpu_percent(interval=None)
            ram        = psutil.virtual_memory()
            temp_c     = read_temperature()
            usage      = collect_cpu_usage(core_count)

            others_cpu = max(0.0, total_cpu - sum(usage.values()))

            profile_data = {
                "system": {
                    "cpu_total":    round(total_cpu, 1),
                    "ram_percent":  round(ram.percent, 1),
                    "ram_used_mb":  round(ram.used / 1024 / 1024, 0),
                    "temperature":  round(temp_c, 1)
                },
                "cpu_services": {
                    **{k: round(v, 1) for k, v in usage.items()},
                    "others": round(others_cpu, 1)
                },
                "ai": {  # ข้อมูล AI จาก stream_manager โดยตรง
                    "enabled":      ai_stats_cache.get('detection_enabled', False),
                    "mode":         ai_stats_cache.get('mode', '-'),
                    "inference_ms": ai_stats_cache.get('inference_ms', 0.0)
                }
            }

            pub.publish(json.dumps(profile_data))

        except Exception as e:
            rospy.logwarn_throttle(5, f"Monitor error: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        pi_system_monitor_and_profiler()
    except rospy.ROSInterruptException:
        pass