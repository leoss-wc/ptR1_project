#!/usr/bin/env python3
"""
analyze_rosbag_70m.py
=====================
วิเคราะห์ rosbag จากการทดสอบเดิน straight path 10m จำนวน 10 ครั้ง

Topics ที่ใช้:
  /odom หรือ /odometry/filtered  → path_length, travel_time
  /move_base/status               → success/failure
  /move_base/result               → goal result code
  /move_base/recovery_status      → recovery count (ถ้ามี)
  /amcl_pose หรือ /robot_pose     → goal_error (ตำแหน่งสุดท้าย vs goal)
  /move_base/goal หรือ /move_base/current_goal → planned goal position

การใช้งาน:
  python3 analyze_rosbag_70m.py --bags trial_1.bag trial_2.bag ...
  python3 analyze_rosbag_70m.py --bags_dir ./bags/
  python3 analyze_rosbag_70m.py --demo   (รันด้วยข้อมูลตัวอย่าง)
"""

import argparse
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyBboxPatch
import warnings
warnings.filterwarnings('ignore')

# ────────────────────────────────────────────────────────────
#  CONFIG
# ────────────────────────────────────────────────────────────
ODOM_TOPICS       = ['/odom', '/odometry/filtered', '/robot_odom']
STATUS_TOPIC      = '/move_base/status'
RESULT_TOPIC      = '/move_base/result'
RECOVERY_TOPICS   = ['/move_base/recovery_status', '/recovery_status']
POSE_TOPICS       = ['/amcl_pose', '/robot_pose', '/base_pose_ground_truth']
GOAL_TOPICS       = ['/move_base/goal', '/move_base/current_goal', '/move_base_simple/goal']

PLANNED_DIST      = 70.0   # เมตร
ODOM_DIST_THRESH  = 0.05   # เมตร — กรอง micro-movement
SUCCESS_CODE      = 3      # move_base GoalStatus: SUCCEEDED = 3

# ────────────────────────────────────────────────────────────
#  ROSBAG PARSER
# ────────────────────────────────────────────────────────────
def find_topic(bag, candidates):
    """หา topic แรกที่มีใน bag จาก list"""
    available = set(bag.get_type_and_topic_info().topics.keys())
    for t in candidates:
        if t in available:
            return t
    return None

def parse_bag(bag_path: str) -> dict:
    """
    อ่าน bag 1 ไฟล์ คืน dict ของ metrics:
      travel_time, path_length, success, recovery, goal_error
    """
    try:
        import rosbag
    except ImportError:
        raise ImportError("ต้องติดตั้ง rosbag: pip install rosbag  หรือใช้ ROS Python environment")

    result = {
        'travel_time': None,
        'path_length': None,
        'success': 0,
        'recovery': 0,
        'goal_error': None,
        'bag_name': os.path.basename(bag_path),
    }

    with rosbag.Bag(bag_path, 'r') as bag:
        available_topics = set(bag.get_type_and_topic_info().topics.keys())

        # ── 1. ODOM → path_length + travel_time ──────────────────
        odom_topic = find_topic(bag, ODOM_TOPICS)
        odom_positions = []
        odom_times     = []

        if odom_topic:
            for _, msg, t in bag.read_messages(topics=[odom_topic]):
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                odom_positions.append((x, y))
                odom_times.append(t.to_sec())

        if len(odom_positions) >= 2:
            dist = 0.0
            for i in range(1, len(odom_positions)):
                dx = odom_positions[i][0] - odom_positions[i-1][0]
                dy = odom_positions[i][1] - odom_positions[i-1][1]
                d  = math.hypot(dx, dy)
                if d > ODOM_DIST_THRESH:  # กรอง noise
                    dist += d
            result['path_length'] = round(dist, 3)
            result['travel_time'] = round(odom_times[-1] - odom_times[0], 2)

        # ── 2. RESULT → success ───────────────────────────────────
        if RESULT_TOPIC in available_topics:
            for _, msg, _ in bag.read_messages(topics=[RESULT_TOPIC]):
                status_code = msg.status.status
                if status_code == SUCCESS_CODE:
                    result['success'] = 1

        # fallback: ดู STATUS_TOPIC
        elif STATUS_TOPIC in available_topics:
            last_status = None
            for _, msg, _ in bag.read_messages(topics=[STATUS_TOPIC]):
                if msg.status_list:
                    last_status = msg.status_list[-1].status
            if last_status == SUCCESS_CODE:
                result['success'] = 1

        # ── 3. RECOVERY ───────────────────────────────────────────
        rec_topic = find_topic(bag, RECOVERY_TOPICS)
        if rec_topic:
            recovery_msgs = list(bag.read_messages(topics=[rec_topic]))
            result['recovery'] = len(recovery_msgs)  # นับจำนวนครั้ง

        # ── 4. GOAL ERROR ─────────────────────────────────────────
        # หา goal position
        goal_x, goal_y = None, None
        goal_topic = find_topic(bag, GOAL_TOPICS)
        if goal_topic:
            for _, msg, _ in bag.read_messages(topics=[goal_topic]):
                try:
                    goal_x = msg.goal.target_pose.pose.position.x
                    goal_y = msg.goal.target_pose.pose.position.y
                except AttributeError:
                    try:
                        goal_x = msg.pose.position.x
                        goal_y = msg.pose.position.y
                    except AttributeError:
                        pass
                break  # ใช้ goal แรกพอ

        # หา final pose
        pose_topic = find_topic(bag, POSE_TOPICS)
        final_x, final_y = None, None
        if pose_topic:
            for _, msg, _ in bag.read_messages(topics=[pose_topic]):
                try:
                    final_x = msg.pose.pose.position.x
                    final_y = msg.pose.pose.position.y
                except AttributeError:
                    try:
                        final_x = msg.pose.position.x
                        final_y = msg.pose.position.y
                    except AttributeError:
                        pass
            # ใช้ค่าสุดท้ายที่ loop วนถึง
        else:
            # fallback: ใช้ตำแหน่งสุดท้ายจาก odom
            if odom_positions:
                final_x, final_y = odom_positions[-1]

        if goal_x is not None and final_x is not None:
            result['goal_error'] = round(math.hypot(final_x - goal_x, final_y - goal_y), 4)
        elif odom_positions:
            # ประมาณ error จากระยะทางที่เดินได้กับ planned
            if result['path_length']:
                result['goal_error'] = round(abs(result['path_length'] - PLANNED_DIST), 4)

    return result


def parse_all_bags(bag_paths: list) -> dict:
    """วน parse ทุก bag แล้วรวมเป็น data dict"""
    all_results = []
    for i, path in enumerate(sorted(bag_paths)):
        print(f"  [{i+1:2d}/{len(bag_paths)}] {os.path.basename(path)} ... ", end='', flush=True)
        try:
            r = parse_bag(path)
            all_results.append(r)
            print(f"✓  time={r['travel_time']}s  dist={r['path_length']}m  success={r['success']}")
        except Exception as e:
            print(f"✗  ERROR: {e}")
            all_results.append({
                'bag_name':    os.path.basename(path),
                'travel_time': None, 'path_length': None,
                'success': 0,       'recovery': 0,
                'goal_error': None,
            })

    n = len(all_results)
    data = {
        'trials':       list(range(1, n + 1)),
        'success':      [r['success']      for r in all_results],
        'travel_time':  [r['travel_time']  for r in all_results],
        'path_length':  [r['path_length']  for r in all_results],
        'planned_dist': PLANNED_DIST,
        'recovery':     [r['recovery']     for r in all_results],
        'goal_error':   [r['goal_error']   for r in all_results],
        'bag_names':    [r['bag_name']     for r in all_results],
    }
    return data


# ────────────────────────────────────────────────────────────
#  DEMO DATA (ใส่ข้อมูลจริงแทนได้)
# ────────────────────────────────────────────────────────────
def get_demo_data():
    return {
        'trials':       list(range(1, 5)),
        'success':      [1, 1, 1, 1, ],
        'travel_time':  [318.69, 310.94, 322.44, 298.99],
        'path_length':  [70.0, 70.0, 70.0, 70.0],
        'planned_dist': 70.0,
        'recovery':     [0, 0, 0, 0],
        'goal_error':   [0.4019, 0.4481, 0.36, 0.4212],
    }


# ────────────────────────────────────────────────────────────
#  STATISTICS
# ────────────────────────────────────────────────────────────
def compute_stats(data: dict) -> dict:
    trials  = data['trials']
    n       = len(trials)
    success = data['success']
    tt      = [v for v in data['travel_time'] if v is not None]
    pl      = [v for v in data['path_length'] if v is not None]
    ge      = [v for v in data['goal_error']  if v is not None]
    rec     = data['recovery']
    planned = data['planned_dist']

    path_error = [abs(p - planned) for p in pl]

    stats = {
        'n':               n,
        'success_count':   sum(success),
        'success_rate':    sum(success) / n * 100,
        'recovery_total':  sum(rec),
        'recovery_trials': sum(1 for r in rec if r > 0),

        'time_mean':  np.mean(tt),  'time_std':  np.std(tt),
        'time_min':   np.min(tt),   'time_max':  np.max(tt),

        'path_mean':  np.mean(pl),  'path_std':  np.std(pl),
        'path_min':   np.min(pl),   'path_max':  np.max(pl),

        'path_err_mean': np.mean(path_error),
        'path_err_std':  np.std(path_error),

        'goal_err_mean': np.mean(ge) if ge else None,
        'goal_err_std':  np.std(ge)  if ge else None,
        'goal_err_max':  np.max(ge)  if ge else None,

        'avg_speed': planned / np.mean(tt) if tt else None,

        'path_errors': path_error,
    }
    return stats


def print_report(data: dict, stats: dict):
    print("\n" + "═"*60)
    print("  NAVIGATION TEST REPORT — 70m Straight Path")
    print("═"*60)
    print(f"  Trials            : {stats['n']}")
    print(f"  Success Rate      : {stats['success_count']}/{stats['n']} ({stats['success_rate']:.0f}%)")
    print(f"  Recovery Events   : {stats['recovery_total']} times in {stats['recovery_trials']} trials")
    print()
    print(f"  Travel Time       : {stats['time_mean']:.2f} ± {stats['time_std']:.2f} s"
          f"  [{stats['time_min']:.1f} – {stats['time_max']:.1f}]")
    print(f"  Path Length       : {stats['path_mean']:.3f} ± {stats['path_std']:.3f} m"
          f"  [{stats['path_min']:.2f} – {stats['path_max']:.2f}]")
    print(f"  Path Error        : {stats['path_err_mean']:.3f} ± {stats['path_err_std']:.3f} m"
          f"  (planned {data['planned_dist']} m)")
    if stats['goal_err_mean'] is not None:
        print(f"  Goal Position Err : {stats['goal_err_mean']:.3f} ± {stats['goal_err_std']:.3f} m"
              f"  (max {stats['goal_err_max']:.3f})")
    if stats['avg_speed']:
        print(f"  Avg Speed         : {stats['avg_speed']:.3f} m/s")
    print("═"*60 + "\n")



# ────────────────────────────────────────────────────────────
#  THESIS COLOUR PALETTE & STYLE
# ────────────────────────────────────────────────────────────
C_BAR_OK   = '#2166ac'   # navy blue  — success bars
C_BAR_FAIL = '#d6604d'   # brick red  — failure bars
C_BAR_NEUT = '#4393c3'   # mid blue   — neutral bars
C_LINE     = '#b2182b'   # dark red   — mean / reference lines
C_PLAN     = '#444444'   # dark grey  — planned distance line
C_GRID     = '#cccccc'   # light grey — grid
C_TEXT     = '#111111'   # near-black — labels / ticks
C_PATCH    = '#d1e5f0'   # pale blue  — ±1σ band

plt.rcParams.update({
    'font.family':       'serif',
    'font.size':         10,
    'axes.titlesize':    11,
    'axes.labelsize':    10,
    'xtick.labelsize':   9,
    'ytick.labelsize':   9,
    'legend.fontsize':   9,
    'figure.dpi':        150,
    'axes.spines.top':   False,
    'axes.spines.right': False,
})


def _style_ax(ax, title: str, xlabel: str, ylabel: str):
    ax.set_facecolor('white')
    ax.set_title(title, color=C_TEXT, fontsize=11, pad=6, fontweight='bold', loc='left')
    ax.set_xlabel(xlabel, color=C_TEXT)
    ax.set_ylabel(ylabel, color=C_TEXT)
    ax.tick_params(axis='both', colors=C_TEXT)
    for spine in ['bottom', 'left']:
        ax.spines[spine].set_edgecolor('#888888')
        ax.spines[spine].set_linewidth(0.8)
    ax.grid(True, axis='y', color=C_GRID, linestyle='--', linewidth=0.6, alpha=0.9)
    ax.set_axisbelow(True)


# ────────────────────────────────────────────────────────────
#  PLOT
# ────────────────────────────────────────────────────────────

# ────────────────────────────────────────────────────────────
#  PLOT  — Table + 3 key charts
# ────────────────────────────────────────────────────────────
def plot_analysis(data: dict, stats: dict, save_path: str = None, excluded_info: list = None):
    """
    Layout (A4-friendly):
      Row 0 : Data table  (full width)
      Row 1 : (a) Travel Time  |  (f) Avg Speed  |  (g) Time Distribution
    """
    trials  = data['trials']
    success = data['success']
    tt      = data['travel_time']
    pl      = data['path_length']
    ge      = data['goal_error']
    rec     = data['recovery']
    planned = data['planned_dist']
    n       = len(trials)

    colors = [C_BAR_OK if s == 1 else C_BAR_FAIL for s in success]
    speeds = [planned / t if t else 0 for t in tt]

    bar_kw  = dict(edgecolor='white', linewidth=0.6, zorder=3)
    mean_kw = dict(color=C_LINE, linestyle='--', linewidth=1.4, zorder=4)
    leg_kw  = dict(frameon=True, framealpha=0.9, edgecolor=C_GRID,
                   facecolor='white', fontsize=9)

    fig = plt.figure(figsize=(13, 9), facecolor='white')
    gs  = gridspec.GridSpec(
        2, 3,
        figure=fig,
        height_ratios=[1.15, 1.6],
        hspace=0.55,
        wspace=0.38,
        top=0.91, bottom=0.07, left=0.07, right=0.97,
    )

    # ── Figure title ─────────────────────────────────────────
    excl_note = ""
    if excluded_info:
        excl_note = "  (excluded: " + ", ".join([f"Trial {e['trial']}" for e in excluded_info]) + ")"

    fig.text(0.5, 0.975,
             'Performance Evaluation: 70 m Straight-Line Navigation',
             ha='center', va='top', color=C_TEXT,
             fontsize=13, fontweight='bold', fontfamily='serif')
    fig.text(0.5, 0.957,
             f"n = {stats['n']} trials  |  Planned distance = {planned} m{excl_note}",
             ha='center', va='top', color='#444444', fontsize=10, fontfamily='serif')

    # ════════════════════════════════════════════════════════
    #  ROW 0 — Data Table (spans all 3 columns)
    # ════════════════════════════════════════════════════════
    ax_tbl = fig.add_subplot(gs[0, :])
    ax_tbl.axis('off')
    ax_tbl.set_title('Table 1  Raw Data — 70 m Straight-Line Navigation Trials',
                     color=C_TEXT, fontsize=11, fontweight='bold',
                     loc='left', pad=6)

    # Build table data — ตัด Path Length, Path Error, Recovery ออก
    col_labels = ['Trial', 'Result', 'Travel Time (s)',
                  'Goal Error (m)', 'Avg Speed (m/s)']

    table_rows = []
    for i in range(n):
        ge_val = f"{ge[i]:.3f}" if ge[i] is not None else '—'
        row = [
            str(trials[i]),
            'Success' if success[i] == 1 else 'Failure',
            f"{tt[i]:.2f}"  if tt[i] is not None else '—',
            ge_val,
            f"{speeds[i]:.3f}",
        ]
        table_rows.append(row)

    # Summary row
    table_rows.append([
        'Mean ± σ',
        f"{stats['success_rate']:.0f}%",
        f"{stats['time_mean']:.2f} ± {stats['time_std']:.2f}",
        f"{stats['goal_err_mean']:.3f} ± {stats['goal_err_std']:.3f}" if stats['goal_err_mean'] else '—',
        f"{stats['avg_speed']:.3f}" if stats['avg_speed'] else '—',
    ])

    tbl = ax_tbl.table(
        cellText=table_rows,
        colLabels=col_labels,
        loc='center',
        cellLoc='center',
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    tbl.scale(1, 1.45)

    # Style header
    for j in range(len(col_labels)):
        cell = tbl[0, j]
        cell.set_facecolor('#2166ac')
        cell.set_text_props(color='white', fontweight='bold')

    # Style data rows
    for i in range(1, len(table_rows) + 1):
        is_summary = (i == len(table_rows))
        for j in range(len(col_labels)):
            cell = tbl[i, j]
            if is_summary:
                cell.set_facecolor('#dce9f5')
                cell.set_text_props(fontweight='bold', color=C_TEXT)
            elif success[i - 1] == 0 and not is_summary:
                cell.set_facecolor('#fde8e4')
                cell.set_text_props(color=C_TEXT)
            else:
                cell.set_facecolor('white' if i % 2 == 1 else '#f5f9fd')
                cell.set_text_props(color=C_TEXT)
            cell.set_edgecolor('#dddddd')

    # ════════════════════════════════════════════════════════
    #  ROW 1 — Three key charts
    # ════════════════════════════════════════════════════════

    # ── (a) Travel Time per Trial ────────────────────────────
    ax1 = fig.add_subplot(gs[1, 0])
    ax1.bar(trials, tt, color=colors, **bar_kw)
    ax1.axhline(stats['time_mean'],
                label=f"Mean = {stats['time_mean']:.2f} s", **mean_kw)
    ax1.fill_between(trials,
                     stats['time_mean'] - stats['time_std'],
                     stats['time_mean'] + stats['time_std'],
                     alpha=0.18, color=C_PATCH, zorder=2,
                     label=f"\u00b11\u03c3 = {stats['time_std']:.2f} s")
    _style_ax(ax1, '(a) Travel Time per Trial', 'Trial', 'Time (s)')
    ax1.set_xticks(trials)
    ax1.legend(**leg_kw)

    # ── (f) Average Speed per Trial ──────────────────────────
    ax2 = fig.add_subplot(gs[1, 1])
    ax2.plot(trials, speeds, 'o-', color=C_BAR_OK, linewidth=1.8,
             markersize=6, markerfacecolor='white',
             markeredgecolor=C_BAR_OK, markeredgewidth=1.5, zorder=4)
    ax2.axhline(np.mean(speeds),
                label=f"Mean = {np.mean(speeds):.3f} m/s", **mean_kw)
    _style_ax(ax2, '(b) Average Speed per Trial', 'Trial', 'Speed (m/s)')
    ax2.set_xticks(trials)
    ax2.legend(**leg_kw)

    # ── (g) Travel Time Distribution ────────────────────────
    ax3 = fig.add_subplot(gs[1, 2])
    ax3.hist(tt, bins=6, color=C_BAR_NEUT, edgecolor='white', linewidth=0.6, zorder=3)
    ax3.axvline(stats['time_mean'],
                label=f"\u03bc = {stats['time_mean']:.2f} s", **mean_kw)
    ax3.axvline(stats['time_mean'] - stats['time_std'],
                color=C_LINE, linestyle=':', linewidth=1.0, alpha=0.6, zorder=4)
    ax3.axvline(stats['time_mean'] + stats['time_std'],
                color=C_LINE, linestyle=':', linewidth=1.0, alpha=0.6, zorder=4,
                label=f"\u03c3 = {stats['time_std']:.2f} s")
    _style_ax(ax3, '(c) Travel Time Distribution', 'Time (s)', 'Frequency')
    ax3.grid(True, axis='both', color=C_GRID, linestyle='--', linewidth=0.6, alpha=0.9)
    ax3.legend(**leg_kw)

    # ── Legend (success / failure) ────────────────────────────
    from matplotlib.patches import Patch
    fig.legend(
        handles=[Patch(facecolor=C_BAR_OK,   label='Success', edgecolor='white'),
                 Patch(facecolor=C_BAR_FAIL, label='Failure', edgecolor='white')],
        loc='lower right', bbox_to_anchor=(0.99, 0.005), **leg_kw)

    # ── Save ─────────────────────────────────────────────────
    if save_path:
        out = save_path
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        out = os.path.join(script_dir, 'navigation_analysis_70m.png')

    plt.savefig(out, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"  \u2192 \u0e1a\u0e31\u0e19\u0e17\u0e36\u0e01\u0e23\u0e39\u0e1b\u0e17\u0e35\u0e48: {out}")
    plt.show()


def detect_outliers_iqr(values: list, k: float = 1.5) -> list:
    arr   = np.array([v if v is not None else np.nan for v in values], dtype=float)
    valid = arr[~np.isnan(arr)]
    if len(valid) < 4:
        return []
    q1, q3 = np.percentile(valid, 25), np.percentile(valid, 75)
    iqr = q3 - q1
    lo, hi = q1 - k * iqr, q3 + k * iqr
    return [i for i, v in enumerate(values) if v is not None and (v < lo or v > hi)]


def filter_data(data: dict, exclude_trials: list):
    if not exclude_trials:
        return data, []
    excluded_info = []
    keep_idx = [i for i, t in enumerate(data['trials']) if t not in exclude_trials]
    removed  = [i for i, t in enumerate(data['trials']) if t in exclude_trials]
    for i in removed:
        excluded_info.append({'trial': data['trials'][i],
                               'travel_time': data['travel_time'][i],
                               'reason': 'manual'})
    new_data = {k: [data[k][i] for i in keep_idx]
                for k in ['trials', 'success', 'travel_time',
                          'path_length', 'recovery', 'goal_error']}
    new_data['planned_dist'] = data['planned_dist']
    if 'bag_names' in data:
        new_data['bag_names'] = [data['bag_names'][i] for i in keep_idx]
    return new_data, excluded_info


# ────────────────────────────────────────────────────────────
#  MAIN
# ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Analyze rosbag — 10m navigation trials')
    parser.add_argument('--bags',         nargs='+', help='ระบุ .bag ไฟล์โดยตรง')
    parser.add_argument('--bags_dir',     type=str,  help='โฟลเดอร์ที่เก็บ .bag ไฟล์')
    parser.add_argument('--demo',         action='store_true', help='รันด้วยข้อมูลตัวอย่าง')
    parser.add_argument('--save',         type=str,  default=None, help='พาธบันทึกรูป .png')
    parser.add_argument('--exclude',      nargs='+', type=int, default=[],
                        help='Trial number ที่ต้องการตัดออก เช่น --exclude 1 3')
    parser.add_argument('--auto-outlier', action='store_true',
                        help='ตัด outlier อัตโนมัติด้วย IQR')
    args = parser.parse_args()

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║   Navigation Rosbag Analyzer — 70m Straight Path    ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    if args.demo:
        print("  [DEMO MODE] ใช้ข้อมูลตัวอย่าง")
        data = get_demo_data()
    elif args.bags or args.bags_dir:
        bag_paths = args.bags or sorted([
            os.path.join(args.bags_dir, f)
            for f in os.listdir(args.bags_dir) if f.endswith('.bag')
        ])
        if not bag_paths:
            print("  ✗ ไม่พบไฟล์ .bag"); return
        print(f"  พบ {len(bag_paths)} ไฟล์ กำลังอ่าน...\n")
        data = parse_all_bags(bag_paths)
    else:
        print("  ไม่มี argument — รัน demo อัตโนมัติ")
        data = get_demo_data()

    exclude_set = set(args.exclude)
    if args.auto_outlier:
        oidx = detect_outliers_iqr(data['travel_time'])
        auto = {data['trials'][i] for i in oidx}
        if auto:
            print(f"  [Auto Outlier] Trial ผิดปกติ: {sorted(auto)}")
            exclude_set |= auto
        else:
            print("  [Auto Outlier] ไม่พบ outlier")

    if exclude_set:
        data, excluded_info = filter_data(data, sorted(exclude_set))
        print(f"  ⚠  ตัด trial: {sorted(exclude_set)}  → เหลือ {len(data['trials'])} trials\n")
    else:
        excluded_info = []

    stats = compute_stats(data)
    print_report(data, stats)
    plot_analysis(data, stats, save_path=args.save, excluded_info=excluded_info)


if __name__ == '__main__':
    main()