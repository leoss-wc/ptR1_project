#!/usr/bin/env python3
import rospy
import psutil
import json
from std_msgs.msg import String

tracked_procs = {}

def categorize_process(p):
    try:
        name = p.name().lower()
        cmdline = " ".join(p.cmdline()).lower()
        
        if 'slam_gmapping' in name or 'gmapping' in cmdline: return 'gmapping'
        elif 'move_base' in name or 'move_base' in cmdline: return 'move_base'
        elif 'amcl' in name or 'amcl' in cmdline: return 'amcl'
        elif 'map_server' in name or 'map_server' in cmdline: return 'map_server'
        elif 'ydlidar' in name or 'ydlidar' in cmdline: return 'ydlidar'
        elif 'rosbridge' in name or 'rosbridge' in cmdline: return 'rosbridge'
        elif 'tf2_web_republisher' in name or 'tf2_web_republisher' in cmdline: return 'tf2_web'
        elif 'ffmpeg' in name or 'ffmpeg' in cmdline: return 'ffmpeg'
        elif 'mediamtx' in name or 'mediamtx' in cmdline or 'rtsp-simple-server' in cmdline: return 'mediamtx'
        elif 'rosserial' in name or 'rosserial' in cmdline: return 'rosserial'
        elif 'tailscale' in name or 'tailscaled' in name: return 'tailscale'
        elif '__name:=' in cmdline: return 'ros_nodes'
        else: return 'others'
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        return 'others'

def pi_system_monitor_and_profiler():
    rospy.init_node('pi_system_monitor', anonymous=True)

    pub_profile = rospy.Publisher('/pi/system_profile', String, queue_size=5)

    core_count = psutil.cpu_count() or 4
    rate = rospy.Rate(1) # 1 Hz

    rospy.loginfo("Raspberry Pi JSON Profiler Started (1 Hz) -> Topic: /pi/system_profile")
    psutil.cpu_percent(interval=None)

    while not rospy.is_shutdown():
        try:
            # 1. System Overall
            total_cpu = psutil.cpu_percent(interval=None)
            ram_percent = psutil.virtual_memory().percent
            
            temp_c = 0.0
            try:
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    temp_c = float(f.read()) / 1000.0
            except FileNotFoundError:
                pass

            # 2. Reset Usage Dictionary
            usage = {
                'gmapping': 0.0, 'move_base': 0.0, 'amcl': 0.0, 'map_server': 0.0,
                'ydlidar': 0.0, 'rosbridge': 0.0, 'tf2_web': 0.0, 'ffmpeg': 0.0,
                'mediamtx': 0.0, 'rosserial': 0.0, 'tailscale': 0.0, 'ros_nodes': 0.0
            }

            # 3. Track Processes
            for pid in psutil.pids():
                if pid not in tracked_procs:
                    try:
                        p = psutil.Process(pid)
                        cat = categorize_process(p)
                        if cat != 'others':
                            p.cpu_percent(interval=None)
                            tracked_procs[pid] = {'proc': p, 'cat': cat}
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass

            dead_pids = []
            for pid, info in tracked_procs.items():
                try:
                    p = info['proc']
                    cat = info['cat']
                    cpu_pct = p.cpu_percent(interval=None) / core_count
                    if cat in usage:
                        usage[cat] += cpu_pct
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    dead_pids.append(pid)

            for pid in dead_pids:
                del tracked_procs[pid]

            others_cpu = total_cpu - sum(usage.values())
            if others_cpu < 0: others_cpu = 0.0
            profile_data = {
                "system": {
                    "cpu_total": round(total_cpu, 1),
                    "ram_percent": round(ram_percent, 1),
                    "temperature": round(temp_c, 1)
                },
                "cpu_services": {
                    **{k: round(v, 1) for k, v in usage.items()},
                    "others": round(others_cpu, 1)
                }
            }

            # 5. แปลงเป็น String และ Publish
            json_str = json.dumps(profile_data)
            pub_profile.publish(json_str)

        except Exception as e:
            rospy.logwarn_throttle(5, f"Monitor error: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        pi_system_monitor_and_profiler()
    except rospy.ROSInterruptException:
        pass