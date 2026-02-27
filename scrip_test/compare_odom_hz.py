import rosbag
import matplotlib.pyplot as plt
import numpy as np

def extract_bag_data(bag_file, topic_name):
    timestamps = []
    print(f"Reading {bag_file}...")
    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                timestamps.append(t.to_sec())
    except Exception as e:
        print(f"Error reading {bag_file}: {e}")
        return [], [], []

    if len(timestamps) < 2:
        return [], [], []

    # คำนวณ Time Delta และ Frequency
    time_deltas = np.diff(timestamps)
    frequencies = 1.0 / time_deltas
    relative_times = np.array(timestamps[1:]) - timestamps[0]
    
    return relative_times, time_deltas, frequencies

TOPIC = '/odom'

# ดึงข้อมูลจากทั้ง 2 สภาวะ
t_idle, dt_idle, hz_idle = extract_bag_data('odom_idle.bag', TOPIC)
t_load, dt_load, hz_load = extract_bag_data('odom_load.bag', TOPIC)

if len(t_idle) == 0 or len(t_load) == 0:
    print("Not enough data in one or both bag files. Please check the files.")
    exit()

# ตั้งค่าการพล็อตกราฟ
plt.figure(figsize=(14, 8))

# --- กราฟที่ 1: เปรียบเทียบ Time Delta (Jitter / ความหน่วง) ---
plt.subplot(2, 1, 1)
plt.plot(t_idle, dt_idle, color='blue', alpha=0.6, label=f'Idle (Mean: {np.mean(dt_idle):.4f} s)')
plt.plot(t_load, dt_load, color='red', alpha=0.6, label=f'Full Load (Mean: {np.mean(dt_load):.4f} s)')
plt.title('Odom Time Delta Comparison: Idle vs Full Load')
plt.ylabel('Time Delta (seconds)')
plt.grid(True)
plt.legend()

# --- กราฟที่ 2: เปรียบเทียบ Frequency (Hz) ---
plt.subplot(2, 1, 2)
plt.plot(t_idle, hz_idle, color='blue', alpha=0.6, label=f'Idle (Mean: {np.mean(hz_idle):.2f} Hz)')
plt.plot(t_load, hz_load, color='red', alpha=0.6, label=f'Full Load (Mean: {np.mean(hz_load):.2f} Hz)')
plt.title('Odom Frequency Comparison: Idle vs Full Load')
plt.xlabel('Time (seconds)')
plt.ylabel('Frequency (Hz)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()