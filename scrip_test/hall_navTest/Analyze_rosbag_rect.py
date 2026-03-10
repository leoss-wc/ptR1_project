#!/usr/bin/env python3
"""
analyze_rosbag_rect.py
========================
วิเคราะห์ rosbag จากการทดสอบเดิน rectangle path 70×2 m (4 goals) จำนวน 1 trial

Topics ที่ใช้:
  /odom หรือ /odometry/filtered  → path_length, travel_time, per-segment time
  /move_base/result               → success per goal
  /move_base/status               → goal status (fallback)
  /move_base/recovery_status      → recovery count per goal
  /amcl_pose หรือ /robot_pose     → goal_error per goal
  /move_base/goal                 → goal positions (G1–G4)

Rectangle layout (top-down):
  G2 ──────────────── G3
  │    (70 m long)     │
  G1 ──────────────── G4  (start/end ≈ G1)
        (2 m wide)

การใช้งาน:
  python3 analyze_rosbag_rect.py --bag trial.bag --side 10
  python3 analyze_rosbag_rect.py --demo
  python3 analyze_rosbag_rect.py --demo --side 5
"""

import argparse
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import warnings
warnings.filterwarnings('ignore')

# ────────────────────────────────────────────────────────────
#  CONFIG
# ────────────────────────────────────────────────────────────
ODOM_TOPICS      = ['/odom', '/odometry/filtered', '/robot_odom']
RESULT_TOPIC     = '/move_base/result'
STATUS_TOPIC     = '/move_base/status'
RECOVERY_TOPICS  = ['/move_base/recovery_status', '/recovery_status']
POSE_TOPICS      = ['/amcl_pose', '/robot_pose', '/base_pose_ground_truth']
GOAL_TOPICS      = ['/move_base/goal', '/move_base_simple/goal']

RECT_LENGTH      = 70.0   # เมตร — ด้านยาว (default)
RECT_WIDTH       =  2.0   # เมตร — ด้านสั้น (default)
ODOM_DIST_THRESH = 0.05   # เมตร — กรอง micro-movement
SUCCESS_CODE     = 3      # move_base GoalStatus: SUCCEEDED = 3
N_GOALS          = 4

# ────────────────────────────────────────────────────────────
#  THESIS COLOUR PALETTE & STYLE
# ────────────────────────────────────────────────────────────
C_BAR_OK   = '#2166ac'
C_BAR_FAIL = '#d6604d'
C_BAR_NEUT = '#4393c3'
C_LINE     = '#b2182b'
C_GRID     = '#cccccc'
C_TEXT     = '#111111'
C_PATCH    = '#d1e5f0'
C_ARROW    = '#555555'

GOAL_COLORS = ['#2166ac', '#1a9850', '#d73027', '#f46d43']  # G1–G4
GOAL_LABELS = ['Goal 1', 'Goal 2', 'Goal 3', 'Goal 4']

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
#  ROSBAG PARSER
# ────────────────────────────────────────────────────────────
def find_topic(bag, candidates):
    available = set(bag.get_type_and_topic_info().topics.keys())
    for t in candidates:
        if t in available:
            return t
    return None


def parse_rect_bag(bag_path: str, length: float = 70.0, width: float = 2.0) -> dict:
    """
    อ่าน bag ที่มี topics:
      /odom                       → path_length, total_time
      /amcl_pose                  → starting position (map frame) + goal error
      /move_base/result           → success per goal + arrival timestamps
      /move_base/recovery_status  → recovery events

    ไม่มี /move_base/goal → คำนวณ goal_positions จาก:
      - amcl_pose แรก = G1 (starting corner)
      - G2 = G1 + (0, width)
      - G3 = G1 + (length, width)
      - G4 = G1 + (length, 0)

    Goal error = distance(amcl_pose ณ arrival time, goal_position)
    """
    try:
        import rosbag
    except ImportError:
        raise ImportError("ต้องใช้ ROS Python environment ที่มี rosbag")

    result = {
        'length':            length,
        'width':             width,
        'planned_perimeter': 2 * (length + width),
        'goal_success':      [0] * N_GOALS,
        'goal_error':        [None] * N_GOALS,
        'goal_time':         [None] * N_GOALS,
        'goal_recovery':     [0] * N_GOALS,
        'goal_positions':    [None] * N_GOALS,
        'total_time':        None,
        'path_length':       None,
        'overall_success':   0,
    }

    with rosbag.Bag(bag_path, 'r') as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())

        # ── 1. ODOM → path_length + total_time ───────────────
        # ลด threshold เป็น 0.005m เพราะ odom ~20Hz → step ~0.025m
        odom_data = []
        if '/odom' in available:
            for _, msg, t in bag.read_messages(topics=['/odom']):
                odom_data.append((t.to_sec(),
                                  msg.pose.pose.position.x,
                                  msg.pose.pose.position.y))

        if len(odom_data) >= 2:
            dist = 0.0
            for i in range(1, len(odom_data)):
                dx = odom_data[i][1] - odom_data[i-1][1]
                dy = odom_data[i][2] - odom_data[i-1][2]
                d  = math.hypot(dx, dy)
                if d > 0.005:          # threshold ลดลงจาก 0.05 → 0.005
                    dist += d
            result['path_length'] = round(dist, 3)
            result['total_time']  = round(odom_data[-1][0] - odom_data[0][0], 2)

        # ── 2. AMCL_POSE timeline ─────────────────────────────
        amcl_data = []   # (t_sec, x, y)
        if '/amcl_pose' in available:
            for _, msg, t in bag.read_messages(topics=['/amcl_pose']):
                amcl_data.append((t.to_sec(),
                                  msg.pose.pose.position.x,
                                  msg.pose.pose.position.y))

        # ── 3. GOAL POSITIONS จาก starting amcl_pose ─────────
        # ใช้ amcl_pose แรกสุดเป็น G1 (origin ของ rectangle ใน map frame)
        if amcl_data:
            x0, y0 = amcl_data[0][1], amcl_data[0][2]
        else:
            x0, y0 = 0.0, 0.0

        result['goal_positions'] = [
            (x0,          y0),           # G1 — จุดเริ่ม
            (x0,          y0 + width),   # G2 — ข้ามด้านสั้น
            (x0 + length, y0 + width),   # G3 — ปลายด้านยาว
            (x0 + length, y0),           # G4 — กลับ
        ]

        # ── 4. RESULT events → success + arrival timestamps ───
        # กรองเฉพาะ SUCCEEDED(3) / ABORTED(4) — ละเว้น intermediate
        result_events = []
        if '/move_base/result' in available:
            for _, msg, t in bag.read_messages(topics=['/move_base/result']):
                try:
                    code = msg.status.status
                except AttributeError:
                    continue
                if code in (2, 3, 4):   # terminal states เท่านั้น
                    result_events.append((t.to_sec(), code))

        # ถ้ามีมากกว่า 4 events (goal ถูก preempt/cancel ก่อนหน้า)
        # ใช้แค่ 4 events สุดท้ายที่สำคัญ หรือกรอง SUCCEEDED ก่อน
        succeeded = [(t, c) for t, c in result_events if c == SUCCESS_CODE]
        # ถ้า SUCCEEDED ครบ 4 ใช้ทั้งหมด ไม่งั้นใช้ terminal 4 ตัวสุดท้าย
        if len(succeeded) == N_GOALS:
            final_events = succeeded
        else:
            final_events = result_events[-N_GOALS:] if len(result_events) >= N_GOALS else result_events

        t_bag_start = odom_data[0][0] if odom_data else None

        for i, (t_arr, code) in enumerate(final_events[:N_GOALS]):
            if code == SUCCESS_CODE:
                result['goal_success'][i] = 1

        if all(s == 1 for s in result['goal_success']):
            result['overall_success'] = 1

        # ── 5. SEGMENT TIME ───────────────────────────────────
        if final_events and t_bag_start is not None:
            prev_t = t_bag_start
            for i, (t_arr, _) in enumerate(final_events[:N_GOALS]):
                seg_t = t_arr - prev_t
                # sanity check: segment ไม่ควรน้อยกว่า 1s หรือมากกว่า total_time
                if 1.0 <= seg_t <= (result['total_time'] or 9999):
                    result['goal_time'][i] = round(seg_t, 2)
                prev_t = t_arr

        # ── 6. GOAL ERROR จาก amcl_pose ณ arrival time ────────
        if amcl_data and final_events:
            for i, (t_arr, _) in enumerate(final_events[:N_GOALS]):
                closest = min(amcl_data, key=lambda p: abs(p[0] - t_arr))
                rx, ry = closest[1], closest[2]
                gx, gy = result['goal_positions'][i]
                result['goal_error'][i] = round(math.hypot(rx - gx, ry - gy), 4)

        # ── 7. RECOVERY per segment ───────────────────────────
        if '/move_base/recovery_status' in available and final_events and t_bag_start is not None:
            rec_times = []
            for _, _, t in bag.read_messages(topics=['/move_base/recovery_status']):
                rec_times.append(t.to_sec())

            seg_starts = [t_bag_start] + [t for t, _ in final_events[:N_GOALS-1]]
            seg_ends   = [t for t, _ in final_events[:N_GOALS]]
            for rec_t in rec_times:
                for i, (s, e) in enumerate(zip(seg_starts, seg_ends)):
                    if s <= rec_t <= e:
                        result['goal_recovery'][i] += 1
                        break

    return result



# ────────────────────────────────────────────────────────────
#  DEMO DATA
# ────────────────────────────────────────────────────────────
def get_demo_data(length: float = 70.0, width: float = 2.0) -> dict:
    perimeter = 2 * (length + width)
    return {
        'length':            length,
        'width':             width,
        'planned_perimeter': perimeter,
        'goal_success':      [1, 1, 1, 1],
        'goal_error':        [0.10, 0.13, 0.11, 0.15],   # เมตร
        'goal_time':         [142.5, 5.2, 144.1, 5.8],   # วินาที/segment (ยาว-สั้น-ยาว-สั้น)
        'goal_recovery':     [0, 0, 1, 0],
        'goal_positions':    [
            (0.0,    0.0),     # G1 — start corner
            (0.0,    width),   # G2 — near corner (ข้ามด้านสั้น)
            (length, width),   # G3 — far corner
            (length, 0.0),     # G4 — far corner (ข้ามด้านสั้น)
        ],
        'total_time':        round(perimeter / 0.493, 1),
        'path_length':       round(perimeter * 1.01, 2),
        'overall_success':   1,
    }


# ────────────────────────────────────────────────────────────
#  STATISTICS
# ────────────────────────────────────────────────────────────
def compute_stats(data: dict) -> dict:
    ge   = [v for v in data['goal_error'] if v is not None]
    gt   = [v for v in data['goal_time']  if v is not None]
    length = data['length']
    width  = data['width']

    return {
        'overall_success':    data['overall_success'],
        'goals_reached':      sum(data['goal_success']),
        'total_time':         data['total_time'],
        'path_length':        data['path_length'],
        'planned_perimeter':  data['planned_perimeter'],
        'path_error':         round(abs(data['path_length'] - data['planned_perimeter']), 3)
                              if data['path_length'] else None,
        'avg_speed':          round(data['path_length'] / data['total_time'], 3)
                              if (data['path_length'] and data['total_time']) else None,
        'goal_err_mean':      round(np.mean(ge), 4) if ge else None,
        'goal_err_std':       round(np.std(ge), 4)  if ge else None,
        'goal_err_max':       round(np.max(ge), 4)  if ge else None,
        'goal_time_mean':     round(np.mean(gt), 2) if gt else None,
        'recovery_total':     sum(data['goal_recovery']),
        'length':             length,
        'width':              width,
    }


def print_report(data: dict, stats: dict):
    print("\n" + "═" * 62)
    print("  NAVIGATION TEST REPORT — Rectangle Path 70×2 m (4 Goals)")
    print("═" * 62)
    print(f"  Rectangle         : {stats['length']} × {stats['width']} m  (perimeter = {stats['planned_perimeter']} m)")
    print(f"  Overall Success   : {'YES ✓' if stats['overall_success'] else 'NO ✗'}  "
          f"({stats['goals_reached']}/4 goals reached)")
    print(f"  Recovery Events   : {stats['recovery_total']}")
    print()
    print(f"  Total Time        : {stats['total_time']} s")
    print(f"  Path Length       : {stats['path_length']} m  (planned {stats['planned_perimeter']} m)")
    if stats['path_error'] is not None:
        print(f"  Path Error        : {stats['path_error']} m")
    if stats['avg_speed']:
        print(f"  Avg Speed         : {stats['avg_speed']} m/s")
    print()
    print(f"  Goal Error (mean) : {stats['goal_err_mean']} ± {stats['goal_err_std']} m  "
          f"(max {stats['goal_err_max']})")
    print()
    print("  Per-Goal Breakdown:")
    for i in range(N_GOALS):
        ge_str  = f"{data['goal_error'][i]:.3f} m" if data['goal_error'][i] is not None else "—"
        gt_str  = f"{data['goal_time'][i]:.1f} s"  if data['goal_time'][i] is not None else "—"
        rec_str = f"{data['goal_recovery'][i]}"
        ok_str  = "✓" if data['goal_success'][i] else "✗"
        print(f"    G{i+1}: {ok_str}  time={gt_str}  error={ge_str}  recovery={rec_str}")
    print("═" * 62 + "\n")


# ────────────────────────────────────────────────────────────
#  PLOT
# ────────────────────────────────────────────────────────────
def plot_analysis(data: dict, stats: dict, save_path: str = None):
    length = data['length']
    width  = data['width']
    side   = length  # alias for diagram scaling
    g_ok   = data['goal_success']
    g_err  = data['goal_error']
    g_time = data['goal_time']
    g_rec  = data['goal_recovery']
    g_pos  = data['goal_positions']
    xlabels = GOAL_LABELS
    x       = np.arange(N_GOALS)
    bar_colors = [C_BAR_OK if s == 1 else C_BAR_FAIL for s in g_ok]
    bar_kw  = dict(edgecolor='white', linewidth=0.6, zorder=3, width=0.55)
    leg_kw  = dict(frameon=True, framealpha=0.9, edgecolor=C_GRID,
                   facecolor='white', fontsize=9)

    # ── Layout: 2 rows × 3 cols ──────────────────────────────
    fig = plt.figure(figsize=(13, 10), facecolor='white')
    gs  = gridspec.GridSpec(
        2, 3,
        figure=fig,
        height_ratios=[1.1, 1.5],
        hspace=0.52, wspace=0.38,
        top=0.91, bottom=0.07, left=0.07, right=0.97,
    )

    # ── Figure title ─────────────────────────────────────────
    ok_str = "Success ✓" if stats['overall_success'] else "Failure ✗"
    ok_col = '#1a6b2f' if stats['overall_success'] else '#a00000'
    fig.text(0.5, 0.975,
             f'Performance Evaluation: {length}×{width} m Rectangle Path Navigation',
             ha='center', va='top', color=C_TEXT,
             fontsize=13, fontweight='bold', fontfamily='serif')
    fig.text(0.5, 0.957,
             f"1 trial  |  4 goals  |  Planned perimeter = {stats['planned_perimeter']} m  |  "
             f"Overall: ",
             ha='center', va='top', color='#444444', fontsize=10, fontfamily='serif')
    # colour the result word separately
    fig.text(0.755, 0.957, ok_str,
             ha='left', va='top', color=ok_col, fontsize=10,
             fontfamily='serif', fontweight='bold')

    # ════════════════════════════════════════════════════════
    #  ROW 0 — Summary Table (spans all 3 cols)
    # ════════════════════════════════════════════════════════
    ax_tbl = fig.add_subplot(gs[0, :])
    ax_tbl.axis('off')
    ax_tbl.set_title('Table 1  Per-Goal Performance Summary',
                     color=C_TEXT, fontsize=11, fontweight='bold', loc='left', pad=6)

    col_labels = ['Goal', 'Result', 'Segment Time (s)',
                  'Goal Error (m)', 'Recovery', 'Cumul. Time (s)']

    table_rows = []
    cumul = 0.0
    for i in range(N_GOALS):
        t_val  = g_time[i] if g_time[i] is not None else 0.0
        cumul += t_val
        table_rows.append([
            f'Goal {i+1}',
            'Reached ✓' if g_ok[i] else 'Failed ✗',
            f"{g_time[i]:.2f}"  if g_time[i] is not None else '—',
            f"{g_err[i]:.3f}"   if g_err[i]  is not None else '—',
            str(g_rec[i]),
            f"{cumul:.2f}",
        ])

    # Summary row
    table_rows.append([
        'Total / Mean',
        f"{stats['goals_reached']}/4",
        f"{stats['goal_time_mean']:.2f} (mean)" if stats['goal_time_mean'] else '—',
        f"{stats['goal_err_mean']:.3f} ± {stats['goal_err_std']:.3f}"
            if stats['goal_err_mean'] is not None else '—',
        str(stats['recovery_total']),
        f"{stats['total_time']:.2f}",
    ])

    tbl = ax_tbl.table(
        cellText=table_rows,
        colLabels=col_labels,
        loc='center',
        cellLoc='center',
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.5)
    tbl.scale(1, 1.6)

    for j in range(len(col_labels)):
        cell = tbl[0, j]
        cell.set_facecolor('#2166ac')
        cell.set_text_props(color='white', fontweight='bold')

    for i in range(1, N_GOALS + 1):
        is_summary = (i == N_GOALS + 1)
        for j in range(len(col_labels)):
            cell = tbl[i, j]
            if i == N_GOALS + 1:
                cell.set_facecolor('#dce9f5')
                cell.set_text_props(fontweight='bold')
            elif g_ok[i - 1] == 0:
                cell.set_facecolor('#fde8e4')
            else:
                cell.set_facecolor('white' if i % 2 == 1 else '#f5f9fd')
            cell.set_edgecolor('#dddddd')

    # summary row style
    for j in range(len(col_labels)):
        cell = tbl[N_GOALS + 1, j]
        cell.set_facecolor('#dce9f5')
        cell.set_text_props(fontweight='bold', color=C_TEXT)
        cell.set_edgecolor('#dddddd')

    # ════════════════════════════════════════════════════════
    #  ROW 1 — 3 key charts
    # ════════════════════════════════════════════════════════

    # ── (a) Goal Position Error per Goal ────────────────────
    ax1 = fig.add_subplot(gs[1, 0])
    ge_vals = [v if v is not None else 0 for v in g_err]
    bars = ax1.bar(x, ge_vals, color=bar_colors, **bar_kw)
    if stats['goal_err_mean'] is not None:
        ax1.axhline(stats['goal_err_mean'], color=C_LINE, linestyle='--', linewidth=1.4,
                    zorder=4, label=f"Mean = {stats['goal_err_mean']:.3f} m")
    _style_ax(ax1, '(a) Goal Position Error', 'Goal', 'Error (m)')
    ax1.set_xticks(x); ax1.set_xticklabels(xlabels)
    ax1.legend(**leg_kw)
    # value labels on bars
    for bar, val in zip(bars, ge_vals):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.003,
                 f'{val:.3f}', ha='center', va='bottom', fontsize=8, color=C_TEXT)

    # ── (b) Segment Travel Time per Goal ────────────────────
    ax2 = fig.add_subplot(gs[1, 1])
    gt_vals = [v if v is not None else 0 for v in g_time]
    bars2 = ax2.bar(x, gt_vals, color=bar_colors, **bar_kw)
    if stats['goal_time_mean'] is not None:
        ax2.axhline(stats['goal_time_mean'], color=C_LINE, linestyle='--', linewidth=1.4,
                    zorder=4, label=f"Mean = {stats['goal_time_mean']:.2f} s")
    _style_ax(ax2, '(b) Segment Travel Time', 'Goal', 'Time (s)')
    ax2.set_xticks(x); ax2.set_xticklabels(xlabels)
    ax2.legend(**leg_kw)
    for bar, val in zip(bars2, gt_vals):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.2,
                 f'{val:.1f}', ha='center', va='bottom', fontsize=8, color=C_TEXT)

    # ── (c) Square Path Diagram ──────────────────────────────
    ax3 = fig.add_subplot(gs[1, 2])
    ax3.set_facecolor('white')
    ax3.set_title('(c) Rectangle Path Diagram', color=C_TEXT,
                  fontsize=11, pad=6, fontweight='bold', loc='left')
    ax3.set_aspect('auto')
    ax3.tick_params(colors=C_TEXT)
    for spine in ['bottom', 'left', 'top', 'right']:
        ax3.spines[spine].set_edgecolor('#cccccc')
    ax3.grid(True, color=C_GRID, linestyle='--', linewidth=0.5, alpha=0.7)


    # วาด planned rectangle (dashed)
    sq_x = [0, 0, length, length, 0]
    sq_y = [0, width, width, 0, 0]
    ax3.plot(sq_x, sq_y, '--', color='#aaaaaa', linewidth=1.2,
             zorder=2, label='Planned path')

    # วาด actual path + goal markers (ถ้ามี goal_positions ครบ)
    gpos = data['goal_positions']
    if all(p is not None for p in gpos):
        px = [p[0] for p in gpos] + [gpos[0][0]]
        py = [p[1] for p in gpos] + [gpos[0][1]]
        ax3.plot(px, py, '-', color=C_BAR_OK, linewidth=1.8,
                 zorder=3, label='Actual path', alpha=0.75)

        for i, (gx, gy) in enumerate(gpos):
            color  = GOAL_COLORS[i]
            marker = '*' if g_ok[i] else 'x'
            ms     = 14  if g_ok[i] else 12
            ax3.plot(gx, gy, marker, color=color, markersize=ms, zorder=5)
            err_txt = f'\n({g_err[i]:.2f} m)' if g_err[i] is not None else ''
            ax3.annotate(
                f'G{i+1}{err_txt}',
                xy=(gx, gy),
                xytext=(gx + length * 0.04, gy + width * 0.8),
                fontsize=8, color=color, fontweight='bold',
                arrowprops=dict(arrowstyle='-', color=color, lw=0.8),
            )
    else:
        # ไม่มี actual path — วาด estimated positions แทน
        est_pos = [(0, 0), (0, width), (length, width), (length, 0)]
        for i, (gx, gy) in enumerate(est_pos):
            ax3.plot(gx, gy, 'o', color=GOAL_COLORS[i], markersize=10, zorder=5, alpha=0.5)
            ax3.text(gx + length * 0.02, gy + width * 0.5,
                     f'G{i+1} (est.)', fontsize=8,
                     color=GOAL_COLORS[i], fontweight='bold')

    # Start annotation
    ax3.annotate('Start', xy=(sq_x[0], sq_y[0]),
                 xytext=(sq_x[0] - length * 0.08, sq_y[0] - width * 1.2),
                 fontsize=8, color='#333333',
                 arrowprops=dict(arrowstyle='->', color='#555555', lw=1.0))

    ax3.set_xlim(-length * 0.12, length * 1.2)
    ax3.set_ylim(-width * 3, width * 5)
    ax3.set_xlabel('X (m)', color=C_TEXT)
    ax3.set_ylabel('Y (m)', color=C_TEXT)

    # เรียก legend เฉพาะตอนมี labeled artists
    handles, labels = ax3.get_legend_handles_labels()
    if handles:
        ax3.legend(handles=handles, labels=labels, loc='lower right', **leg_kw)

    # ── Overall metrics text box ──────────────────────────────
    info_lines = [
        f"Total time : {stats['total_time']} s",
        f"Path length: {stats['path_length']} m",
        f"Path error : {stats['path_error']} m",
        f"Avg speed  : {stats['avg_speed']} m/s",
        f"Recovery   : {stats['recovery_total']} event(s)",
    ]
    info_txt = '\n'.join(info_lines)
    fig.text(0.5, 0.01, info_txt,
             ha='center', va='bottom', fontsize=8.5, color='#333333',
             fontfamily='serif',
             bbox=dict(boxstyle='round,pad=0.4', facecolor='#f5f9fd',
                       edgecolor='#cccccc', alpha=0.9))

    # ── Save ─────────────────────────────────────────────────
    if save_path:
        out = save_path
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        out = os.path.join(script_dir, 'navigation_analysis_square.png')

    plt.savefig(out, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"  → บันทึกรูปที่: {out}")
    plt.show()


# ────────────────────────────────────────────────────────────
#  MAIN
# ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Analyze rosbag — rectangle path 70×2 m navigation (4 goals, 1 trial)')
    parser.add_argument('--bag',    type=str,   help='ไฟล์ .bag ที่ต้องการวิเคราะห์')
    parser.add_argument('--length', type=float, default=70.0, help='ด้านยาว (เมตร) default=70')
    parser.add_argument('--width',  type=float, default=2.0,  help='ด้านสั้น (เมตร) default=2')
    parser.add_argument('--demo',   action='store_true', help='รันด้วยข้อมูลตัวอย่าง')
    parser.add_argument('--save',   type=str,   default=None, help='พาธบันทึกรูป .png')
    args = parser.parse_args()

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  Navigation Rosbag Analyzer — Rect Path (4 Goals)   ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    if args.demo:
        print(f"  [DEMO MODE]  {args.length}×{args.width} m")
        data = get_demo_data(length=args.length, width=args.width)
    elif args.bag:
        print(f"  อ่าน: {args.bag}  ({args.length}×{args.width} m)")
        data = parse_rect_bag(args.bag, length=args.length, width=args.width)
    else:
        print("  ไม่มี argument — รัน demo อัตโนมัติ  (70×2 m)")
        data = get_demo_data(length=args.length, width=args.width)

    stats = compute_stats(data)
    print_report(data, stats)
    plot_analysis(data, stats, save_path=args.save)


if __name__ == '__main__':
    main()