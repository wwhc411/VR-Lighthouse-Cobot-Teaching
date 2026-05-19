"""
论文级轨迹处理流程四阶段对比可视化
====================================

方案一：1×4 水平流程图，统一视角
方案二：Z轴叠层瀑布图（单轴，最紧凑）

各子图说明：
  (a) 原始轨迹     → 灰色虚线，体现采集噪声
  (b) 预平滑后     → 橙色实线，高斯去噪后的平滑结果
  (c) 重采样后     → 紫色实线 + 均匀标记点，体现空间均匀采样
  (d) MLS配准后    → 红色粗实线，与目标几乎重合，标注 RMSE

目标轨迹：始终为宝蓝色粗实线（参考基准，贯穿全图）

使用方法：
  1. 将此文件与 tunable_registration.py 放在同一目录
  2. 直接运行：python plot_pipeline_paper.py
  3. 输出：trajectory_pipeline_paper.pdf / .png（同目录）

依赖：
  - tunable_registration.py（同目录）
  - numpy, matplotlib, scipy（已在 tunable_registration 中使用）
"""

import sys
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401（注册3D投影）

# ============================================================
#  路径配置：确保能 import tunable_registration
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import tunable_registration as tr

# ============================================================
#  中文字体 & 负号修复
# ============================================================
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'STSong', 'KaiTi']
plt.rcParams['axes.unicode_minus'] = False

# ============================================================
#  ⭐ 用户可调参数
# ============================================================

# 数据路径（自动从 tunable_registration 读取，也可手动覆盖）
SOURCE_CSV = tr.SOURCE_CSV
TARGET_CSV = tr.TARGET_CSV

# 输出文件路径
SAVE_PDF = os.path.join(SCRIPT_DIR, "trajectory_pipeline_paper.pdf")
SAVE_PNG = os.path.join(SCRIPT_DIR, "trajectory_pipeline_paper.png")

# 统一 3D 视角（所有子图相同）
VIEW_ELEV = 25   # 仰角
VIEW_AZIM = -50  # 方位角

# 各阶段源轨迹颜色（(a)~(d)）
STAGE_COLORS = ["#313131", '#FF7F0E', "#884AC2", '#D62728']

# 目标轨迹颜色（固定，贯穿四幅子图）
TARGET_COLOR = '#1F77B4'

# (c) 重采样后：均匀标记点的采样密度（每隔 N 个点画一个标记）
# 设为 None 则自动按总点数的约 1/50 计算
MARKER_EVERY = None

# 3D 图交互控制（中键）
MIDDLE_MOUSE_INTERACTION = {
    "enable": True,
    "rotate_sensitivity": 0.35,  # 中键拖拽旋转灵敏度
    "zoom_sensitivity": 0.010,   # Ctrl+中键拖拽缩放灵敏度
}

# ============================================================
#  ⭐ 方案二专用参数（Z 轴叠层瀑布图）
# ============================================================

# 各阶段沿 Z 方向的偏移间距 (mm)。
# 正值 = 向上叠放；None = 自动按轨迹 Z 方向跨度的 60% 计算。
WATERFALL_Z_OFFSET = None

# 方案二的 3D 视角
WATERFALL_ELEV = 20
WATERFALL_AZIM = -55

# 方案二输出路径（SOURCE / TARGET 分开保存）
SAVE_WATERFALL_SOURCE_PDF = os.path.join(SCRIPT_DIR, "trajectory_waterfall_source_paper.pdf")
SAVE_WATERFALL_SOURCE_PNG = os.path.join(SCRIPT_DIR, "trajectory_waterfall_source_paper.png")
SAVE_WATERFALL_TARGET_PDF = os.path.join(SCRIPT_DIR, "trajectory_waterfall_target_paper.pdf")
SAVE_WATERFALL_TARGET_PNG = os.path.join(SCRIPT_DIR, "trajectory_waterfall_target_paper.png")

# 重采样后 SOURCE/TARGET 同图误差对比输出路径
SAVE_RESAMPLED_COMPARE_PDF = os.path.join(SCRIPT_DIR, "trajectory_resampled_compare_paper.pdf")
SAVE_RESAMPLED_COMPARE_PNG = os.path.join(SCRIPT_DIR, "trajectory_resampled_compare_paper.png")

# 方案四：原始轨迹 XYZ 单轴对比输出路径（3张图）
SAVE_ORIGINAL_AXIS_X_PDF = os.path.join(SCRIPT_DIR, "trajectory_original_axis_x_compare.pdf")
SAVE_ORIGINAL_AXIS_X_PNG = os.path.join(SCRIPT_DIR, "trajectory_original_axis_x_compare.png")
SAVE_ORIGINAL_AXIS_Y_PDF = os.path.join(SCRIPT_DIR, "trajectory_original_axis_y_compare.pdf")
SAVE_ORIGINAL_AXIS_Y_PNG = os.path.join(SCRIPT_DIR, "trajectory_original_axis_y_compare.png")
SAVE_ORIGINAL_AXIS_Z_PDF = os.path.join(SCRIPT_DIR, "trajectory_original_axis_z_compare.pdf")
SAVE_ORIGINAL_AXIS_Z_PNG = os.path.join(SCRIPT_DIR, "trajectory_original_axis_z_compare.png")

# 各可视化图片生成开关（True=生成，False=跳过）
VIS_GENERATION = {
    "pipeline_1x4": False,       # 方案一：1×4 水平流程图
    "waterfall_source": False,   # 方案二第1张：SOURCE_CSV 三阶段瀑布图
    "waterfall_target": False,   # 方案二第2张：TARGET_CSV 三阶段瀑布图
    "resampled_compare": False,  # 新增：重采样后 SOURCE/TARGET 误差对比图
    "original_axis_compare": True,  # 方案四：原始轨迹 XYZ 三个单轴折线对比图
}


# ============================================================
#  Step 1：运行处理流程，获取各阶段数据
# ============================================================
def run_pipeline():
    """
    复现 tunable_registration 的四阶段处理流程，返回各阶段点云数据。

    返回：
        dict，键值：
          'original_source'     - 原始源轨迹
          'original_target'     - 原始目标轨迹
          'pre_smoothed_source' - 预平滑后源轨迹
          'pre_smoothed_target' - 预平滑后目标轨迹
          'resampled_source'    - 重采样后源轨迹
          'resampled_target'    - 重采样后目标轨迹
          'aligned_source'      - 配准后源轨迹
          'aligned_target'      - 配准后对应目标轨迹
          'errors'              - 配准后逐点误差数组 (mm)
    """
    print("=" * 60)
    print("  轨迹处理流程 - 运行中")
    print("=" * 60)

    # ---- 加载原始数据 ----
    print("\n[1/4] 加载数据")
    source, _ = tr.load_csv(SOURCE_CSV)
    target, _ = tr.load_csv(TARGET_CSV)

    # ---- 预平滑 ----
    print("\n[2/4] 预平滑（Pre-smooth）")
    source_pre = tr.pre_smooth(source, tr.PRE_SMOOTH)
    target_pre = tr.pre_smooth(target, tr.PRE_SMOOTH)
    print(f"  源：{len(source)} → {len(source_pre)} 点")

    # ---- 空间重采样对齐 ----
    print("\n[3/4] 空间重采样对齐（Resample）")
    source_rs, target_rs = tr.align_trajectories(source_pre, target_pre, tr.TIME_ALIGN)
    print(f"  源：{len(source_pre)} → {len(source_rs)} 点")

    # ---- 后平滑（配准前的最终预处理）----
    print("\n[4/4] 后平滑 + 配准（Post-smooth + Registration）")
    source_proc = tr.preprocess(source_rs, tr.POST_SMOOTH)
    target_proc = tr.preprocess(target_rs, tr.POST_SMOOTH)

    # ---- 配准（根据 tunable_registration 中设定的 METHOD 自动选择）----
    method = tr.METHOD
    print(f"  配准方法: {method.upper()}")

    if method == "mls":
        _, aligned, errors, _ = tr.mls_align(source_proc, target_proc, tr.MLS_PARAMS)

    elif method == "kabsch_segmented":
        if tr.SEQUENCE_ALIGN_MODE["enable"]:
            _, aligned, errors, _ = tr.segmented_kabsch_align_by_sequence(
                source_proc, target_proc, tr.SEGMENTED_PARAMS)
        else:
            _, aligned, errors, _ = tr.segmented_kabsch_align(
                source_proc, target_proc, tr.SEGMENTED_PARAMS)

    else:
        # kabsch / kabsch_icp / icp / ransac 等返回变换矩阵
        if method == "kabsch":
            T = tr.kabsch_align(source_proc, target_proc)
        elif method == "kabsch_icp":
            T = tr.kabsch_icp_align(source_proc, target_proc, tr.ICP_PARAMS)
        elif method == "icp":
            T = tr.icp_align(source_proc, target_proc, tr.ICP_PARAMS)
        elif method == "ransac":
            T = tr.ransac_gicp_align(source_proc, target_proc, tr.RANSAC_PARAMS, tr.ICP_PARAMS)
        else:
            raise ValueError(f"未知配准方法：{method}")
        aligned = tr.apply_transform(source_proc, T)
        errors = np.linalg.norm(aligned - target_proc, axis=1)

    rmse = float(np.sqrt(np.mean(errors ** 2)))
    print(f"\n  ✅ 配准完成  RMSE = {rmse:.3f} mm")

    return {
        'original_source':     source,
        'original_target':     target,
        'pre_smoothed_source': source_pre,
        'pre_smoothed_target': target_pre,
        'resampled_source':    source_rs,
        'resampled_target':    target_rs,
        'aligned_source':      aligned,
        'aligned_target':      target_proc,
        'errors':              errors,
    }


# ============================================================
#  辅助函数
# ============================================================
def _unified_limits(stages_data):
    """从所有阶段数据计算统一的 XYZ 坐标范围（等比例正方体）。"""
    pts = [
        v for k, v in stages_data.items()
        if k != 'errors' and isinstance(v, np.ndarray) and v.ndim == 2 and v.shape[1] >= 3
    ]
    all_pts = np.vstack(pts)
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float(np.max(maxs - mins)) * 1.10  # 留 5% 边距
    return center, span


def _set_equal_limits(ax, center, span):
    """为 3D 轴设置等比例坐标范围。"""
    h = span / 2.0
    ax.set_xlim(center[0] - h, center[0] + h)
    ax.set_ylim(center[1] - h, center[1] + h)
    ax.set_zlim(center[2] - h, center[2] + h)
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect((1, 1, 1))


def _scale_3d_limits(ax, scale):
    """按比例缩放 3D 轴范围（scale<1 放大，scale>1 缩小）。"""
    x0, x1 = ax.get_xlim3d()
    y0, y1 = ax.get_ylim3d()
    z0, z1 = ax.get_zlim3d()

    xc, yc, zc = (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    hx, hy, hz = (x1 - x0) / 2.0 * scale, (y1 - y0) / 2.0 * scale, (z1 - z0) / 2.0 * scale

    ax.set_xlim3d(xc - hx, xc + hx)
    ax.set_ylim3d(yc - hy, yc + hy)
    ax.set_zlim3d(zc - hz, zc + hz)


def _enable_middle_mouse_controls(fig, axes3d):
    """
    为 3D 图启用中键交互：
      - 中键拖拽：旋转
      - Ctrl + 中键拖拽：缩放
      - 滚轮：缩放
    """
    if not MIDDLE_MOUSE_INTERACTION.get("enable", True):
        return

    if not isinstance(axes3d, (list, tuple)):
        axes3d = [axes3d]

    axes_set = set(axes3d)
    state = {
        "pressed": False,
        "mode": "rotate",
        "last_x": None,
        "last_y": None,
        "ax": None,
    }

    rot_k = float(MIDDLE_MOUSE_INTERACTION.get("rotate_sensitivity", 0.35))
    zoom_k = float(MIDDLE_MOUSE_INTERACTION.get("zoom_sensitivity", 0.010))

    def _on_press(event):
        if event.button != 2:
            return
        if event.inaxes not in axes_set:
            return
        state["pressed"] = True
        state["ax"] = event.inaxes
        state["last_x"] = event.x
        state["last_y"] = event.y
        key = event.key or ""
        state["mode"] = "zoom" if ("ctrl" in key or "control" in key) else "rotate"

    def _on_release(event):
        if event.button == 2:
            state["pressed"] = False
            state["ax"] = None

    def _on_motion(event):
        if not state["pressed"] or state["ax"] is None:
            return
        if state["last_x"] is None or state["last_y"] is None:
            return

        dx = event.x - state["last_x"]
        dy = event.y - state["last_y"]
        ax = state["ax"]

        if state["mode"] == "rotate":
            ax.azim -= dx * rot_k
            ax.elev -= dy * rot_k
        else:
            scale = 1.0 + dy * zoom_k
            scale = np.clip(scale, 0.80, 1.25)
            _scale_3d_limits(ax, float(scale))

        state["last_x"] = event.x
        state["last_y"] = event.y
        fig.canvas.draw_idle()

    def _on_scroll(event):
        if event.inaxes not in axes_set:
            return
        scale = 0.92 if event.step > 0 else 1.08
        _scale_3d_limits(event.inaxes, scale)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', _on_press)
    fig.canvas.mpl_connect('button_release_event', _on_release)
    fig.canvas.mpl_connect('motion_notify_event', _on_motion)
    fig.canvas.mpl_connect('scroll_event', _on_scroll)


# ============================================================
#  Step 2：绘制论文级 1×4 流程对比图
# ============================================================
def plot_pipeline_figure(stages_data):
    """
    绘制 1×4 水平流程对比图，统一视角，颜色编码各阶段。
    """

    center, span = _unified_limits(stages_data)

    # ---- 四阶段配置 ----
    STAGES = [
        {
            'title':   '(a) 原始轨迹',
            'src_key': 'original_source',
            'tgt_key': 'original_target',
            'color':   STAGE_COLORS[0],
            'lw':      0.9,
            'ls':      '--',    # 虚线 → 视觉上体现噪声抖动
            'alpha':   0.85,
            'markers': True,
            'show_rmse': False,
        },
        {
            'title':   '(b) 预平滑后',
            'src_key': 'pre_smoothed_source',
            'tgt_key': 'pre_smoothed_target',
            'color':   STAGE_COLORS[1],
            'lw':      1.2,
            'ls':      '-',
            'alpha':   0.85,
            'markers': True,
            'show_rmse': False,
        },
        {
            'title':   '(c) 重采样后',
            'src_key': 'resampled_source',
            'tgt_key': 'resampled_target',
            'color':   STAGE_COLORS[2],
            'lw':      1.0,
            'ls':      '-',
            'alpha':   0.85,
            'markers': True,   # 均匀圆点 → 体现空间均匀采样
            'show_rmse': False,
        },
        {
            'title':   '(d) 配准后',
            'src_key': 'aligned_source',
            'tgt_key': 'aligned_target',
            'color':   STAGE_COLORS[3],
            'lw':      1.8,
            'ls':      '-',
            'alpha':   0.90,
            'markers': True,
            'show_rmse': True, # 角落标注 RMSE
        },
    ]

    # ---- 创建画布 ----
    fig = plt.figure(figsize=(22, 6.5), dpi=150)
    fig.patch.set_facecolor('white')

    axes = []
    for col, s in enumerate(STAGES):
        ax = fig.add_subplot(1, 4, col + 1, projection='3d')
        axes.append(ax)

        src = stages_data[s['src_key']]
        tgt = stages_data[s['tgt_key']]
        color = s['color']

        # ── 目标轨迹（蓝色粗实线，参考基准）──
        ax.plot(tgt[:, 0], tgt[:, 1], tgt[:, 2],
                color=TARGET_COLOR, lw=2.0, ls='-', alpha=0.55, zorder=2)

        # ── 源轨迹 ──
        ax.plot(src[:, 0], src[:, 1], src[:, 2],
                color=color, lw=s['lw'], ls=s['ls'],
                alpha=s['alpha'], zorder=3)

        # ── 均匀采样标记点 ──
        n = len(src)
        step = MARKER_EVERY if MARKER_EVERY else max(1, n // 120)
        idx = np.arange(0, n, step)
        ax.scatter(src[idx, 0], src[idx, 1], src[idx, 2],
                   color=color, s=5, zorder=4, alpha=0.90,
                   edgecolors='none', linewidths=0)

        # ── (d) 左上角标注 RMSE ──
        if s['show_rmse']:
            errs = stages_data['errors']
            rmse = float(np.sqrt(np.mean(errs ** 2)))
            mean_e = float(np.mean(errs))
            ax.text2D(0.05, 0.93,
                      f'RMSE = {rmse:.2f} mm\nmean = {mean_e:.2f} mm',
                      transform=ax.transAxes, fontsize=8.5,
                      color=color, fontweight='bold', va='top',
                      bbox=dict(boxstyle='round,pad=0.3',
                                facecolor='white', edgecolor=color,
                                alpha=0.88))

        # ── 坐标轴范围 & 视角 ──
        _set_equal_limits(ax, center, span)
        ax.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)

        # 仅最左子图保留轴标签与刻度
        if col == 0:
            ax.set_xlabel('X (mm)', fontsize=8, labelpad=3)
            ax.set_ylabel('Y (mm)', fontsize=8, labelpad=3)
            ax.set_zlabel('Z (mm)', fontsize=8, labelpad=3)
            ax.tick_params(axis='both', labelsize=6)
        else:
            ax.set_xticklabels([])
            ax.set_yticklabels([])
            ax.set_zticklabels([])
            ax.set_xlabel('')
            ax.set_ylabel('')
            ax.set_zlabel('')

        # ── 淡化背景面板，弱化网格 ──
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor('#C8C8C8')
        ax.grid(True, alpha=0.18)

        # ── 子图标题（粗体，12pt）──
        ax.set_title(s['title'], fontsize=12, fontweight='bold',
                     pad=4, loc='center')

    _enable_middle_mouse_controls(fig, axes)

    # ---- 执行 tight_layout，之后再添加子图间箭头 ----
    plt.tight_layout(rect=[0, 0.10, 1, 0.94])

    # ---- 子图间流程箭头 ----
    # 在 tight_layout 之后读取各子图真实坐标位置
    for i, ax in enumerate(axes[:-1]):
        bb_cur  = ax.get_position()
        bb_next = axes[i + 1].get_position()
        x_arrow = (bb_cur.x1 + bb_next.x0) / 2.0
        y_arrow = (bb_cur.y0 + bb_cur.y1) / 2.0
        fig.text(x_arrow, y_arrow, '→',
                 ha='center', va='center',
                 fontsize=20, color='#666666',
                 fontweight='bold',
                 transform=fig.transFigure)

    # ---- 整图图例（下方居中，横排）----
    legend_items = [
        mpatches.Patch(facecolor=TARGET_COLOR,   alpha=0.65,
                       label='目标轨迹（参考基准）'),
        mpatches.Patch(facecolor=STAGE_COLORS[0],
                       label='(a) 原始　　虚线 = 含噪声'),
        mpatches.Patch(facecolor=STAGE_COLORS[1],
                       label='(b) 预平滑后'),
        mpatches.Patch(facecolor=STAGE_COLORS[2],
                       label='(c) 重采样　圆点 = 均匀采样'),
        mpatches.Patch(facecolor=STAGE_COLORS[3],
                       label='(d) 配准后　标注 RMSE'),
    ]
    fig.legend(handles=legend_items,
               loc='lower center', ncol=5,
               fontsize=9, frameon=True,
               bbox_to_anchor=(0.5, 0.01),
               edgecolor='#AAAAAA', framealpha=0.95)

    # ---- 总标题 ----
    fig.suptitle('点云配准处理流程：原始 → 预平滑 → 重采样 → MLS 配准',
                 fontsize=14, fontweight='bold', y=0.99)

    # ---- 保存 ----
    fig.savefig(SAVE_PDF, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"\n  ✅ PDF 已保存: {SAVE_PDF}")
    fig.savefig(SAVE_PNG, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"  ✅ PNG 已保存: {SAVE_PNG}")

    plt.show()
    print("  ✅ 可视化完成")


# ============================================================
#  方案二：Z 轴叠层瀑布图
# ============================================================
def plot_waterfall_figure(stages_data, trajectory_type='source'):
    """
    方案二：Z 轴叠层瀑布图。

    将三阶段轨迹沿 Z 方向依次偏移叠放在同一个 3D 坐标系中，
    形成"瀑布层叠"效果：

        层 1（最底）：① 原始轨迹（灰色虚线）
        层 2        ：② 预平滑后（橙色实线）
        层 3（最顶）：③ 重采样后（紫色实线 + 均匀圆点）

    层与层之间用半透明竖向连接线（curtain）暗示对应关系。

    参数:
        stages_data: run_pipeline() 返回的阶段数据
        trajectory_type: 'source' 或 'target'
    """

    if trajectory_type not in ('source', 'target'):
        raise ValueError("trajectory_type 必须是 'source' 或 'target'")

    key_suffix = trajectory_type
    csv_path = SOURCE_CSV if trajectory_type == 'source' else TARGET_CSV
    data_label = 'SOURCE_CSV' if trajectory_type == 'source' else 'TARGET_CSV'
    curve_label = 'SOURCE' if trajectory_type == 'source' else 'TARGET'

    # ---- 四阶段层次配置（顺序 = 从下到上）----
    LAYERS = [
        {
            'label':        '① 原始轨迹',
            'src_key':      f'original_{key_suffix}',
            'color':        STAGE_COLORS[0],
            'lw':           1.1,
            'ls':           '--',
            'alpha':        0.92,
            'markers':      True,
            'jitter_effect': True,   # 抖动视觉强化
        },
        {
            'label':   '② 预平滑后',
            'src_key': f'pre_smoothed_{key_suffix}',
            'color':   STAGE_COLORS[1],
            'lw':      1.2,
            'ls':      '-',
            'alpha':   0.90,
            'markers': True,
        },
        {
            'label':   '③ 重采样后',
            'src_key': f'resampled_{key_suffix}',
            'color':   STAGE_COLORS[2],
            'lw':      1.0,
            'ls':      '-',
            'alpha':   0.90,
            'markers': True,
        },
    ]

    # ---- 自动计算 Z 偏移步长 ----
    ref_pts = stages_data[f'original_{key_suffix}']
    z_span = float(ref_pts[:, 2].max() - ref_pts[:, 2].min())
    if z_span < 1.0:
        z_span = 1.0

    dz = float(WATERFALL_Z_OFFSET) if WATERFALL_Z_OFFSET is not None else z_span * 0.60

    # 各层 Z 偏移（0-based：层 0=①原始轨迹，层 2=③重采样后）
    z_base = float(stages_data[f'original_{key_suffix}'][:, 2].min())
    layer_z_offsets = [i * dz for i in range(len(LAYERS))]

    # ---- 辅助：Z 平移 ----
    def _shift_z(pts, delta_z):
        shifted = pts.copy()
        shifted[:, 2] = shifted[:, 2] - z_base + delta_z
        return shifted

    # ---- 画布：GridSpec 分左侧 3D 区（75%）+ 右侧标注栏（25%）----
    from matplotlib.gridspec import GridSpec
    from matplotlib.lines import Line2D

    fig = plt.figure(figsize=(16, 10), dpi=150)
    fig.patch.set_facecolor('white')

    gs = GridSpec(1, 2, width_ratios=[3, 1], figure=fig,
                  left=0.03, right=0.97, bottom=0.06, top=0.90,
                  wspace=0.04)
    ax      = fig.add_subplot(gs[0], projection='3d')
    ax_info = fig.add_subplot(gs[1])
    ax_info.axis('off')            # 右侧仅用于摆放文字，不显示坐标轴

    _enable_middle_mouse_controls(fig, [ax])

    # ---- 各阶段分层（纯轨迹绘制，不在 3D 轴内放任何文字）----
    shifted_layers = []  # 缓存各层偏移后数据，供幕帘连线使用
    for i, layer in enumerate(LAYERS):
        src = stages_data[layer['src_key']]
        z_off = layer_z_offsets[i]
        src_shifted = _shift_z(src, z_off)
        color = layer['color']

        # 主曲线
        ax.plot(src_shifted[:, 0], src_shifted[:, 1], src_shifted[:, 2],
                color=color, lw=layer['lw'], ls=layer['ls'],
                alpha=layer['alpha'], zorder=3 + i)

        # ── ① 原始轨迹：三层抖动视觉增强 ──
        if layer.get('jitter_effect'):
            # 层 A：背景光晕（粗半透明实线，扩展抖动包络视感）
            ax.plot(src_shifted[:, 0], src_shifted[:, 1], src_shifted[:, 2],
                    color=color, lw=7.0, ls='-', alpha=0.08, zorder=2)

            # 层 B：密集散点（每隔少数点一个，展示逐帧采样位置的离散抖动）
            jitter_step = max(1, len(src_shifted) // 400)
            jitter_idx = np.arange(0, len(src_shifted), jitter_step)
            ax.scatter(src_shifted[jitter_idx, 0],
                       src_shifted[jitter_idx, 1],
                       src_shifted[jitter_idx, 2],
                       color=color, s=4, alpha=0.55,
                       zorder=6, edgecolors='none')

            # 层 C：偏差刺脊线（局部平滑基线 → 原始点，形成沿程"刺梳"效果）
            from scipy.ndimage import uniform_filter1d as _uf
            _smooth = src_shifted.copy()
            for _d in range(3):
                _smooth[:, _d] = _uf(src_shifted[:, _d], size=30)
            spike_step = max(1, len(src_shifted) // 110)
            for _si in range(0, len(src_shifted), spike_step):
                ax.plot([_smooth[_si, 0], src_shifted[_si, 0]],
                        [_smooth[_si, 1], src_shifted[_si, 1]],
                        [_smooth[_si, 2], src_shifted[_si, 2]],
                        color=color, lw=1.0, alpha=0.62, zorder=5)

        # ③ 均匀采样标记点（所有层都添加）
        n_pts = len(src_shifted)
        mk_step = MARKER_EVERY if MARKER_EVERY else max(1, n_pts // 120)
        mk_idx = np.arange(0, n_pts, mk_step)
        # ⑨重采样层用白色边框强调均匀性；其他层用无边框弱化
        ec = 'white' if layer['markers'] and not layer.get('jitter_effect') else 'none'
        lw_ec = 0.4 if ec == 'white' else 0.0
        ax.scatter(src_shifted[mk_idx, 0], src_shifted[mk_idx, 1], src_shifted[mk_idx, 2],
                   color=color, s=5, alpha=0.88, zorder=5,
                   edgecolors=ec, linewidths=lw_ec)

        # 半透明幕帘连接线（当前层 → 下方相邻层）
        if i > 0:
            prev_shifted = shifted_layers[i - 1]
            curtain_step = max(1, len(src_shifted) // 40)
            curtain_idx  = np.arange(0, len(src_shifted), curtain_step)
            prev_len = len(prev_shifted)
            for ci in curtain_idx:
                prev_ci = min(int(ci * prev_len / len(src_shifted)), prev_len - 1)
                ax.plot([src_shifted[ci, 0], prev_shifted[prev_ci, 0]],
                        [src_shifted[ci, 1], prev_shifted[prev_ci, 1]],
                        [src_shifted[ci, 2], prev_shifted[prev_ci, 2]],
                        color=color, lw=0.4, alpha=0.14, zorder=1)

        shifted_layers.append(src_shifted)


    # ---- 视角 ----
    ax.view_init(elev=WATERFALL_ELEV, azim=WATERFALL_AZIM)

    # ---- 3D 坐标轴样式 ----
    ax.set_xlabel('X (mm)', fontsize=9, labelpad=4)
    ax.set_ylabel('Y (mm)', fontsize=9, labelpad=4)
    ax.set_zlabel('Z 偏移 (mm)', fontsize=9, labelpad=4)
    ax.tick_params(axis='both', labelsize=7)

    # Z 轴刻度替换为阶段名称
    z_tick_vals   = layer_z_offsets
    z_tick_labels = ['①original', '②pre_smoothed', '③resampled']
    ax.set_zticks(z_tick_vals)
    ax.set_zticklabels(z_tick_labels, fontsize=7)

    # XY 范围等比例
    all_xy = np.vstack(
        [_shift_z(stages_data[l['src_key']], layer_z_offsets[j])[:, :2]
         for j, l in enumerate(LAYERS)]
    )
    x_min, y_min = all_xy.min(axis=0)
    x_max, y_max = all_xy.max(axis=0)
    x_ctr = (x_min + x_max) / 2.0
    y_ctr = (y_min + y_max) / 2.0
    xy_half = max(x_max - x_min, y_max - y_min) * 0.55
    ax.set_xlim(x_ctr - xy_half, x_ctr + xy_half)
    ax.set_ylim(y_ctr - xy_half, y_ctr + xy_half)
    ax.set_zlim(-dz * 0.3, layer_z_offsets[-1] + dz * 0.5)

    # 背景面板弱化
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor('#C0C0C0')
    ax.grid(True, alpha=0.15)

    # ================================================================
    #  右侧标注栏：图例
    #  全部使用 ax_info 坐标系（x: 0~1, y: 0~1），不接触 3D 轴
    # ================================================================

    # ── 标题 ──
    ax_info.text(0.08, 0.97, '图　例',
                 fontsize=12, fontweight='bold', va='top', ha='left',
                 color='#222222',
                 transform=ax_info.transAxes)

    # ── 分隔线 ──
    ax_info.axhline(y=0.93, xmin=0.05, xmax=0.95,
                    color='#BBBBBB', lw=1.0)

    # ── 图例条目：使用真实 Line2D 手柄，体现线型差异 ──
    legend_handles = [
        Line2D([0], [0], color=STAGE_COLORS[0], lw=1.2, ls='--',
               marker='o', markersize=3.5, alpha=0.75,
             label=f'① {curve_label} 原始轨迹\n刺脊线＋散点 = 沿程抖动'),
        Line2D([0], [0], color=STAGE_COLORS[1], lw=1.4, ls='-',
               label='② 预平滑后\n高斯滤波去噪'),
        Line2D([0], [0], color=STAGE_COLORS[2], lw=1.2, ls='-',
               marker='o', markersize=4,
               label='③ 重采样后\n圆点 = 均匀间距'),
    ]
    leg = ax_info.legend(handles=legend_handles,
                         loc='upper left',
                         bbox_to_anchor=(0.02, 0.91),
                         fontsize=9,
                         frameon=True,
                         edgecolor='#CCCCCC',
                         framealpha=0.96,
                         labelspacing=1.1,
                         handlelength=2.4,
                         handleheight=1.0,
                         borderpad=0.9)

    # ── 右侧栏外边框 ──
    for spine in ax_info.spines.values():
        spine.set_visible(False)

    # ---- 总标题（图外顶部，不占轴空间）----
    fig.suptitle(f'点云配准流程：Z 轴叠层瀑布图（{data_label}: {os.path.basename(csv_path)}）\n'
                 '从下到上：① original → ② pre_smoothed → ③ resampled',
                 fontsize=13, fontweight='bold', y=0.98)

    # ---- 底部说明文字 ----
    fig.text(0.50, 0.01,
             f'注：各层沿 Z 轴等间距偏移以分离显示；① {curve_label} 原始轨迹刺脊线体现采集噪声抖动',
             ha='center', va='bottom', fontsize=8.5, color='#666666',
             style='italic')

    # ---- 保存 ----
    save_pdf = SAVE_WATERFALL_SOURCE_PDF if trajectory_type == 'source' else SAVE_WATERFALL_TARGET_PDF
    save_png = SAVE_WATERFALL_SOURCE_PNG if trajectory_type == 'source' else SAVE_WATERFALL_TARGET_PNG
    fig.savefig(save_pdf, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"\n  ✅ 瀑布图 PDF 已保存: {save_pdf}")
    fig.savefig(save_png, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"  ✅ 瀑布图 PNG 已保存: {save_png}")

    plt.show()
    print("  ✅ 瀑布图可视化完成")


# ============================================================
#  方案三：重采样后 SOURCE/TARGET 同图误差对比
# ============================================================
def plot_resampled_comparison_figure(stages_data):
    """
    在同一幅 3D 图中对比重采样后的 SOURCE/TARGET 轨迹，
    并通过逐点误差着色体现轨迹差异。
    """
    src = stages_data['resampled_source']
    tgt = stages_data['resampled_target']

    # 对齐到相同点数，确保逐点误差可计算
    n = min(len(src), len(tgt))
    if len(src) != len(tgt):
        print(f"  ⚠️ 重采样点数不一致，按最小点数截断: SOURCE={len(src)}, TARGET={len(tgt)}, 使用 {n}")
    src = src[:n]
    tgt = tgt[:n]

    errors = np.linalg.norm(src - tgt, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    min_e = float(np.min(errors))
    mean_e = float(np.mean(errors))
    max_e = float(np.max(errors))
    p95_e = float(np.percentile(errors, 95))

    source_color = '#E53935'  # SOURCE_CSV 使用红色
    target_color = '#1E88E5'  # TARGET_CSV 使用蓝色

    fig = plt.figure(figsize=(14, 8), dpi=150)
    fig.patch.set_facecolor('white')
    ax = fig.add_subplot(111, projection='3d')

    _enable_middle_mouse_controls(fig, [ax])

    # TARGET 作为参考基准（蓝色）
    ax.plot(tgt[:, 0], tgt[:, 1], tgt[:, 2],
            color=target_color, lw=2.1, alpha=0.90, label='TARGET_CSV (resampled)', zorder=2)

    # SOURCE 使用重采样阶段颜色
    ax.plot(src[:, 0], src[:, 1], src[:, 2],
            color=source_color, lw=1.7, alpha=0.92, label='SOURCE_CSV (resampled)', zorder=3)

    # 稀疏连线显示两轨迹对应偏差（绿→红，颜色层次简洁）
    from matplotlib.colors import LinearSegmentedColormap, Normalize
    cmap = LinearSegmentedColormap.from_list('vivid_green_red', ['#00E676', '#FFF176', '#FF1744'])
    err_cap = float(np.percentile(errors, 95))
    if err_cap <= (min_e + 1e-12):
        err_cap = min_e + (max(max_e - min_e, 1e-6))
    norm = Normalize(vmin=min_e, vmax=err_cap)

    link_step = max(1, n // 80)
    for i in range(0, n, link_step):
        e_i = min(float(errors[i]), err_cap)
        ax.plot([src[i, 0], tgt[i, 0]],
                [src[i, 1], tgt[i, 1]],
                [src[i, 2], tgt[i, 2]],
                color=cmap(norm(e_i)), lw=0.9, alpha=0.85, zorder=1)

    # 坐标范围统一
    all_pts = np.vstack([src, tgt])
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float(np.max(maxs - mins)) * 1.10
    _set_equal_limits(ax, center, span)
    ax.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)

    ax.set_xlabel('X (mm)', fontsize=9)
    ax.set_ylabel('Y (mm)', fontsize=9)
    ax.set_zlabel('Z (mm)', fontsize=9)
    ax.tick_params(axis='both', labelsize=8)
    ax.grid(True, alpha=0.20)

    # 误差色条（对应彩色连线）
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.02, fraction=0.03)
    cbar.set_label('link deviation (mm)', fontsize=9)
    cbar.ax.tick_params(labelsize=8)

    # 统计信息
    ax.text2D(0.02, 0.98,
              f'RMSE = {rmse:.3f} mm\n'
              f'Min  = {min_e:.3f} mm\n'
              f'Mean = {mean_e:.3f} mm\n'
              f'Max  = {max_e:.3f} mm',
              transform=ax.transAxes, va='top', ha='left',
              fontsize=9, fontweight='bold', color='#222222',
              bbox=dict(boxstyle='round,pad=0.3',
                        facecolor='white', edgecolor='#888888', alpha=0.92))

    ax.legend(loc='upper right', fontsize=9, frameon=True, framealpha=0.95)

    fig.suptitle('重采样后轨迹对比：SOURCE_CSV vs TARGET_CSV（同图误差可视化）',
                 fontsize=13, fontweight='bold', y=0.98)
    fig.text(0.5, 0.01,
             '注：仅显示 SOURCE/TARGET 折线；两轨迹连线按偏差大小以鲜艳绿→红着色',
             ha='center', va='bottom', fontsize=8.5, color='#666666', style='italic')

    fig.savefig(SAVE_RESAMPLED_COMPARE_PDF, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"\n  ✅ 重采样对比图 PDF 已保存: {SAVE_RESAMPLED_COMPARE_PDF}")
    fig.savefig(SAVE_RESAMPLED_COMPARE_PNG, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"  ✅ 重采样对比图 PNG 已保存: {SAVE_RESAMPLED_COMPARE_PNG}")

    plt.show()
    print("  ✅ 重采样对比图可视化完成")


# ============================================================
#  方案四：原始轨迹 XYZ 单轴折线对比（输出3图）
# ============================================================
def plot_original_axis_comparison_figures(stages_data):
    """
    将 SOURCE/TARGET 的原始轨迹按 X/Y/Z 三个坐标轴分别绘制折线对比图，输出3张图片。
    """
    src = stages_data['original_source']
    tgt = stages_data['original_target']

    source_color = '#E53935'
    target_color = '#1E88E5'

    axis_configs = [
        ('X', 0, SAVE_ORIGINAL_AXIS_X_PDF, SAVE_ORIGINAL_AXIS_X_PNG),
        ('Y', 1, SAVE_ORIGINAL_AXIS_Y_PDF, SAVE_ORIGINAL_AXIS_Y_PNG),
        ('Z', 2, SAVE_ORIGINAL_AXIS_Z_PDF, SAVE_ORIGINAL_AXIS_Z_PNG),
    ]

    # 横轴压缩系数：<1 表示压缩横轴显示
    x_compress_ratio = 0.35
    src_idx = np.arange(len(src)) * x_compress_ratio
    tgt_idx = np.arange(len(tgt)) * x_compress_ratio
    common_n = min(len(src), len(tgt))

    for axis_name, dim, save_pdf, save_png in axis_configs:
        # 1:1 正方形画布
        fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
        fig.patch.set_facecolor('white')

        # 主折线
        ax.plot(src_idx, src[:, dim], color=source_color, lw=1.5, alpha=0.92,
                label='SOURCE_CSV original')
        ax.plot(tgt_idx, tgt[:, dim], color=target_color, lw=1.5, alpha=0.92,
                label='TARGET_CSV original')

        # 稀疏采样点强调离散采样特征
        src_step = max(1, len(src) // 180)
        tgt_step = max(1, len(tgt) // 180)
        src_mark_idx = np.arange(0, len(src), src_step)
        tgt_mark_idx = np.arange(0, len(tgt), tgt_step)
        src_mark_x = src_mark_idx * x_compress_ratio
        tgt_mark_x = tgt_mark_idx * x_compress_ratio

        ax.scatter(src_mark_x, src[src_mark_idx, dim], s=8,
                   color=source_color, edgecolors='white', linewidths=0.2, alpha=0.90)
        ax.scatter(tgt_mark_x, tgt[tgt_mark_idx, dim], s=8,
                   facecolors='none', edgecolors=target_color, linewidths=0.6, alpha=0.95)

        # 统计量（按公共长度对齐后计算）
        axis_diff = src[:common_n, dim] - tgt[:common_n, dim]
        mean_abs = float(np.mean(np.abs(axis_diff)))
        rmse = float(np.sqrt(np.mean(axis_diff ** 2)))
        max_abs = float(np.max(np.abs(axis_diff)))

        note = f'|Δ{axis_name}| mean={mean_abs:.3f} mm, RMSE={rmse:.3f} mm, max={max_abs:.3f} mm'
        if len(src) != len(tgt):
            note += f'  (统计使用前 {common_n} 点)'

        ax.text(0.01, 0.97, note,
                transform=ax.transAxes, ha='left', va='top',
                fontsize=8.8, color='#222222',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='white', edgecolor='#999999', alpha=0.90))

        ax.set_xlabel(f'Point Index (x{ x_compress_ratio:.2f} compressed)', fontsize=10)
        ax.set_ylabel(f'{axis_name} (mm)', fontsize=10)
        ax.set_title(f'原始轨迹单轴对比 - {axis_name}轴', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.22, linestyle='--')
        ax.legend(fontsize=9, loc='upper right', frameon=True, framealpha=0.95)

        plt.tight_layout()
        fig.savefig(save_pdf, dpi=300, bbox_inches='tight', facecolor='white')
        fig.savefig(save_png, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"  ✅ {axis_name}轴对比图已保存: {save_pdf}")
        print(f"  ✅ {axis_name}轴对比图已保存: {save_png}")

        plt.show()

    print("  ✅ 方案四可视化完成（X/Y/Z 三图）")


# ============================================================
#  入口
# ============================================================
if __name__ == '__main__':
    if not any(VIS_GENERATION.values()):
        print("未启用任何可视化生成项，请在 VIS_GENERATION 中将至少一项设为 True。")
    else:
        stages = run_pipeline()

        if VIS_GENERATION["pipeline_1x4"]:
            print("\n" + "=" * 60)
            print("  方案一：1×4 水平流程图")
            print("=" * 60)
            plot_pipeline_figure(stages)

        if VIS_GENERATION["waterfall_source"]:
            print("\n" + "=" * 60)
            print("  方案二：Z 轴叠层瀑布图（第1张：SOURCE_CSV）")
            print("=" * 60)
            plot_waterfall_figure(stages, trajectory_type='source')

        if VIS_GENERATION["waterfall_target"]:
            print("\n" + "=" * 60)
            print("  方案二：Z 轴叠层瀑布图（第2张：TARGET_CSV）")
            print("=" * 60)
            plot_waterfall_figure(stages, trajectory_type='target')

        if VIS_GENERATION["resampled_compare"]:
            print("\n" + "=" * 60)
            print("  方案三：重采样后 SOURCE/TARGET 同图误差对比")
            print("=" * 60)
            plot_resampled_comparison_figure(stages)

        if VIS_GENERATION["original_axis_compare"]:
            print("\n" + "=" * 60)
            print("  方案四：原始轨迹 XYZ 单轴折线对比（输出3图）")
            print("=" * 60)
            plot_original_axis_comparison_figures(stages)
