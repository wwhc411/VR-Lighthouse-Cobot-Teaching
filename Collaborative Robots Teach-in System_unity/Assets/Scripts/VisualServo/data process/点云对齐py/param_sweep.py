"""
参数自动扫参工具 (param_sweep.py)
===================================

对 loocv_candidates / delta_d / gaussian_sigma 三个关键参数做网格搜索，
自动完成「配准训练 → 坐标系转换 → 复用变换 → 误差评估」完整流水线，
输出所有参数组合对应的误差矩阵，并打印最优配置。

执行方式：
    python param_sweep.py              # 完整扫描
    python param_sweep.py --quick      # 快速验证（2×2×2=8组）
    python param_sweep.py --resume     # 跳过已完成的组合继续运行

Author: AI Assistant
Date: 2026-03-03
"""

import os
import re
import sys
import csv
import json
import copy
import shutil
import subprocess
import time
import traceback
from itertools import product
from pathlib import Path

# ============================================================================
#                         ⭐ 路径配置 — 按项目实际情况修改
# ============================================================================

# 脚本目录（此文件所在目录）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 四个核心脚本的绝对路径
TUNABLE_REG_PY      = os.path.join(SCRIPT_DIR, "tunable_registration.py")
CONVERT_PY          = os.path.join(SCRIPT_DIR, "convert_segmented_transform.py")
APPLY_PY            = os.path.join(SCRIPT_DIR, "apply_transform.py")

# trajectory_matching_error.py 位于上级目录
DATA_PROCESS_DIR    = os.path.dirname(SCRIPT_DIR)
TRAJ_ERROR_PY       = os.path.join(DATA_PROCESS_DIR, "trajectory_matching_error.py")

# 中间文件 / 输入输出（与各脚本硬编码路径保持一致）
RECORDINGS_DIR      = r"E:\Unity cangku\lighthouse_3.4\Assets\StreamingAssets\曲线"
# apply_transform.py 输出的已变换轨迹（机械臂坐标系下的 tracker 轨迹，作为 teach 参考）
TRANSFORMED_CSV     = os.path.join(RECORDINGS_DIR, "quxian_tracker - tcp_transformed.csv")
# 复用时的实际机械臂 TCP 录制（robot 坐标系，作为 replay 待评估轨迹）
# ⚠️  注意：应使用 "quxian_tcp - tcp"（机械臂坐标系），而非 "tcp_first_period.csv"（灯塔坐标系）
TARGET_CSV          = os.path.join(RECORDINGS_DIR, "quxian_tcp - tcp.csv")

# 扫参结果目录
SWEEP_RESULTS_DIR   = os.path.join(SCRIPT_DIR, "sweep_results")
SWEEP_CSV           = os.path.join(SWEEP_RESULTS_DIR, "sweep_results.csv")
BEST_JSON           = os.path.join(SWEEP_RESULTS_DIR, "best_config.json")
TEMP_DIR            = os.path.join(SWEEP_RESULTS_DIR, "temp")

# 临时副本路径
TEMP_TUNABLE_PY     = os.path.join(TEMP_DIR, "tunable_registration_patched.py")
TEMP_APPLY_PY       = os.path.join(TEMP_DIR, "apply_transform_patched.py")

# ============================================================================
#                         ⭐ 参数网格定义
#
# 说明：
#   - PARAM_GRID_ADAPTIVE_*  → adaptive_spatial 模式专用（需扫 noise_level / noise_suppression_factor）
#   - PARAM_GRID_CHORD_*     → chord_spatial    模式专用（需扫 densify_factor）
#   - 默认全量扫描会合并两类网格，也可用 --adaptive / --chord CLI 参数单独运行
# ============================================================================

# ── adaptive_spatial 完整网格 ──────────────────────────────────────────────
# 3D弦距重采样旧版，依赖预平滑；noise_level/noise_suppression_factor 是其特有参数
PARAM_GRID_ADAPTIVE_FULL = {
    "resample_method":          ["adaptive_spatial"],
    "loocv_candidates":         [[0.15], [0.2], [0.3], [0.25], [0.35], [0.1], [0.4], [0.45]],  # LOOCV候选带宽（相对距离）
    "gaussian_sigma":           [3, 4, 5],               # 预平滑高斯核半宽 (mm)
    "noise_level":              [1, 1.5, 2, 2.5],         # 测量噪声水平 (mm)
    "noise_suppression_factor": [1.5, 2.0, 0.8],        # 噪声抑制因子
    # chord_spatial 专用（此模式填 None 表示不注入）
    "densify_factor":           [None],
    "delta_d":                  [None],
}

# ── chord_spatial 完整网格 ────────────────────────────────────────────────
# 真3D弦距重采样新版，解决海岸线悖论；densify_factor 是其特有参数
PARAM_GRID_CHORD_FULL = {
    "resample_method":          ["chord_spatial"],
    "loocv_candidates":         [None],
    "delta_d":                  [None],   # 弦距步长门限 (mm)
    "gaussian_sigma":           [None],                  # 预平滑高斯核半宽 (mm)
    "densify_factor":           [None],           # 插值加密倍数（chord_spatial专用）
    # adaptive_spatial 专用（此模式填 None 表示不注入）
    "noise_level":              [None],
    "noise_suppression_factor": [None],
}

# ── 合并完整网格（默认）──────────────────────────────────────────────────
PARAM_GRID_FULL = [PARAM_GRID_ADAPTIVE_FULL, PARAM_GRID_CHORD_FULL]

# ── adaptive_spatial 快速验证网格 ──────────────────────────────────────────
PARAM_GRID_ADAPTIVE_QUICK = {
    "resample_method":          ["adaptive_spatial"],
    "loocv_candidates":         [[0.02], [0.05]],
    "delta_d":                  [0.5, 1.0],
    "gaussian_sigma":           [3, 5],
    "noise_level":              [0.3, 0.5],
    "noise_suppression_factor": [1.5, 2.0],
    "densify_factor":           [None],
}

# ── chord_spatial 快速验证网格 ─────────────────────────────────────────────
PARAM_GRID_CHORD_QUICK = {
    "resample_method":          ["chord_spatial"],
    "loocv_candidates":         [[0.02], [0.05]],
    "delta_d":                  [0.5, 1.0],
    "gaussian_sigma":           [3, 5],
    "densify_factor":           [20, 50],
    "noise_level":              [None],
    "noise_suppression_factor": [None],
}

# ── 合并快速验证网格 ──────────────────────────────────────────────────────
PARAM_GRID_QUICK = [PARAM_GRID_ADAPTIVE_QUICK, PARAM_GRID_CHORD_QUICK]


def build_combo_list(grids) -> list:
    """
    将一个或多个方法专用参数网格（dict）展开为统一的参数 dict 列表。
    支持传入单个 dict 或 list[dict]（多方法合并）。
    值为 None 的参数保留在 combo 中（patch 时会自动跳过）。
    """
    if isinstance(grids, dict):
        grids = [grids]
    all_combos = []
    for grid in grids:
        keys = list(grid.keys())
        for vals in product(*grid.values()):
            combo = dict(zip(keys, vals))
            all_combos.append(combo)
    return all_combos

# 各步骤超时时间（秒）
TIMEOUT_TRAIN   = 600   # 训练（LOOCV计算量大）
TIMEOUT_CONVERT = 120   # 坐标系转换
TIMEOUT_APPLY   = 180   # 复用变换
TIMEOUT_EVAL    = 300   # 误差评估（Fréchet距离计算量大）

# CSV 结果字段
RESULT_FIELDS = [
    "combo_id",
    "resample_method",           # 重采样方法 (adaptive_spatial / chord_spatial)
    "loocv_candidates",
    "delta_d",
    "gaussian_sigma",
    "noise_level",               # adaptive_spatial 专用
    "noise_suppression_factor",  # adaptive_spatial 专用
    "densify_factor",            # chord_spatial 专用
    "apd_mean_mm",
    "apd_max_mm",
    "apd_p95_mm",
    "apd_std_mm",
    "frechet_mm",
    "start_error_mm",
    "end_error_mm",
    "grade",
    "elapsed_s",
    "status",
    "error_msg",
]


# ============================================================================
#                        工具函数
# ============================================================================

def ensure_dirs():
    """确保输出目录存在"""
    os.makedirs(SWEEP_RESULTS_DIR, exist_ok=True)
    os.makedirs(TEMP_DIR, exist_ok=True)


def patch_py_file(src_path: str, dst_path: str, patches: dict):
    """
    将 src_path 内容 patch 后写入 dst_path（正则替换字典键值行）。

    patches 格式：
        {
            "loocv_candidates": repr([0.02, 0.05]),   # str 值
            "delta_d":          "1.0",
            "gaussian_sigma":   "5",
        }
    键名是字典中的 key_name，值是新的 Python 值字符串。
    """
    with open(src_path, "r", encoding="utf-8") as f:
        content = f.read()

    for key_name, new_val in patches.items():
        # 匹配形如：  "key_name": <anything_to_end_of_value>,
        # 值可能是数字、列表 [...]，直到行尾或注释位置
        pattern = rf'("{key_name}"|' + rf"'{key_name}')" + r'\s*:\s*[^\n]+'
        replacement = f'"{key_name}": {new_val},'
        new_content, n = re.subn(pattern, replacement, content, count=1)
        if n == 0:
            raise ValueError(
                f"[patch] 在 {os.path.basename(src_path)} 中未找到键: {key_name}\n"
                f"        请确认该键存在于字典配置中。"
            )
        content = new_content

    # 在文件头部注入 matplotlib 非交互后端，防止训练脚本弹窗阻塞
    if "matplotlib" in content and "matplotlib.use(" not in content:
        inject = "import matplotlib\nmatplotlib.use('Agg')  # 扫参时禁用GUI后端\n"
        # 插到第一个 import 行之前
        content = re.sub(r"^(import |from )", inject + r"\1", content, count=1, flags=re.MULTILINE)

    with open(dst_path, "w", encoding="utf-8") as f:
        f.write(content)


def run_script(script_path: str, timeout: int = 300, cwd: str = None) -> tuple:
    """
    执行 Python 脚本，返回 (returncode, stdout, stderr)。
    使用与当前解释器相同的 Python 可执行文件。
    """
    python_exe = sys.executable
    if cwd is None:
        cwd = os.path.dirname(script_path)
    # 强制子进程使用 UTF-8 编码 stdout/stderr，避免 Windows GBK 导致 UnicodeEncodeError
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"         # Python 3.7+ UTF-8 模式
    try:
        result = subprocess.run(
            [python_exe, script_path],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            cwd=cwd,
            env=env,
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired as e:
        return -1, "", f"[TIMEOUT] 执行超过 {timeout}s: {e}"
    except Exception as e:
        return -2, "", f"[EXCEPTION] {e}"


def load_completed_combos(csv_path: str) -> set:
    """
    从已有结果 CSV 中读取已完成的参数组合，用于中断恢复。
    返回 set of (resample_method, loocv_repr, delta_d, gaussian_sigma,
                        noise_level, noise_suppression_factor, densify_factor)
    """
    completed = set()
    if not os.path.exists(csv_path):
        return completed
    try:
        with open(csv_path, "r", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row.get("status") == "ok":
                    key = (
                        row.get("resample_method", ""),
                        row["loocv_candidates"],
                        row["delta_d"],
                        row["gaussian_sigma"],
                        row.get("noise_level", ""),
                        row.get("noise_suppression_factor", ""),
                        row.get("densify_factor", ""),
                    )
                    completed.add(key)
    except Exception:
        pass
    return completed


def append_result_row(csv_path: str, row: dict):
    """追加写入一行结果到 CSV（边跑边保存，防止中途崩溃丢数据）"""
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=RESULT_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def eval_error_via_subprocess(transformed_csv: str, target_csv: str) -> dict:
    """
    通过子进程运行 trajectory_matching_error.py 并解析输出来获取误差数据。
    作为直接 import 失败时的备用方案。
    """
    rc, out, err = run_script(
        TRAJ_ERROR_PY,
        timeout=TIMEOUT_EVAL,
        cwd=DATA_PROCESS_DIR,
    )
    if rc != 0:
        raise RuntimeError(f"trajectory_matching_error.py 执行失败:\n{err[-500:]}")

    result = {}
    # 解析 stdout 中的关键行
    for line in out.splitlines():
        line = line.strip()
        m = re.search(r"平均误差[：:]\s*([\d.]+)", line)
        if m:
            result.setdefault("apd_mean", float(m.group(1)))
        m = re.search(r"最大误差[：:]\s*([\d.]+)", line)
        if m:
            result.setdefault("apd_max", float(m.group(1)))
        m = re.search(r"P95[：:]\s*([\d.]+)", line)
        if m:
            result.setdefault("apd_p95", float(m.group(1)))
        m = re.search(r"标准差[：:]\s*([\d.]+)", line)
        if m:
            result.setdefault("apd_std", float(m.group(1)))
        m = re.search(r"Fréchet[距离]*[：: ]+\s*([\d.]+)", line)
        if m:
            result.setdefault("frechet", float(m.group(1)))
        m = re.search(r"起点误差[：:]\s*([\d.]+)", line)
        if m:
            result.setdefault("start_error", float(m.group(1)))
        m = re.search(r"终点误差[：:]\s*([\d.]+)", line)
        if m:
            result.setdefault("end_error", float(m.group(1)))
        m = re.search(r"综合评级[：:\s]+([\u4e00-\u9fa5\w ()（）]+)", line)
        if m:
            result.setdefault("grade", m.group(1).strip())

    if "apd_mean" not in result:
        raise RuntimeError(f"无法从输出中解析误差数据\n输出前500字符:\n{out[:500]}")
    return result


def eval_error(transformed_csv: str, target_csv: str) -> dict:
    """
    评估误差：优先直接 import 调用，失败时回退到子进程解析。
    返回标准化字典:
        apd_mean, apd_max, apd_p95, apd_std, frechet,
        start_error, end_error, grade
    """
    try:
        # 动态 import，避免模块级全局状态污染
        if DATA_PROCESS_DIR not in sys.path:
            sys.path.insert(0, DATA_PROCESS_DIR)
        # 每次重新 import（清除缓存，确保无副作用残留）
        if "trajectory_matching_error" in sys.modules:
            del sys.modules["trajectory_matching_error"]

        from trajectory_matching_error import compute_trajectory_matching_error
        res = compute_trajectory_matching_error(
            replay_csv=transformed_csv,
            teach_csv=target_csv,
            visualize=False,
            save_report=False,
        )
        grade_str, _ = res.get("overall_grade_tuple", (res.get("overall_grade", "N/A"), False))
        # 兼容不同返回格式
        if isinstance(res.get("overall_grade"), tuple):
            grade_str = res["overall_grade"][0]
        elif isinstance(res.get("overall_grade"), str):
            grade_str = res["overall_grade"]

        return {
            "apd_mean":    res["apd"]["mean"],
            "apd_max":     res["apd"]["max"],
            "apd_p95":     res["apd"]["p95"],
            "apd_std":     res["apd"]["std"],
            "frechet":     res["frechet"]["frechet_distance"],
            "start_error": res["endpoints"]["start_error"],
            "end_error":   res["endpoints"]["end_error"],
            "grade":       grade_str,
        }
    except Exception as import_err:
        print(f"  [WARN] 直接import失败: {import_err}，回退到子进程解析...")
        return eval_error_via_subprocess(transformed_csv, target_csv)


def format_best_box(row: dict) -> str:
    """格式化最优结果输出框（自动适应方法专用参数）"""
    loocv  = row.get("loocv_candidates", "N/A")
    method = row.get("resample_method", "N/A")
    W = 52   # 框内最大宽度
    bar = "═" * W
    lines = [
        f"╔{bar}╗",
        f"║{'最优参数配置':^{W-2}}║",
        f"╠{bar}╣",
        f"║  resample_method  = {method:<{W-22}}║",
        f"║  loocv_candidates = {str(loocv):<{W-22}}║",
        f"║  delta_d          = {str(row.get('delta_d','N/A')):<5} mm{' '*(W-29)}║",
        f"║  gaussian_sigma   = {str(row.get('gaussian_sigma','N/A')):<5} mm{' '*(W-29)}║",
    ]
    # 方法专用参数
    if method == "adaptive_spatial":
        lines += [
            f"║  noise_level      = {str(row.get('noise_level','N/A')):<{W-22}}║",
            f"║  noise_supp_factor= {str(row.get('noise_suppression_factor','N/A')):<{W-22}}║",
        ]
    elif method == "chord_spatial":
        lines += [
            f"║  densify_factor   = {str(row.get('densify_factor','N/A')):<{W-22}}║",
        ]
    lines += [
        f"╠{bar}╣",
        f"║  APD 均值  = {float(row['apd_mean_mm']):.4f} mm{' '*(W-27)}║",
        f"║  APD 最大  = {float(row['apd_max_mm']):.4f} mm{' '*(W-27)}║",
        f"║  APD P95   = {float(row['apd_p95_mm']):.4f} mm{' '*(W-27)}║",
        f"║  Fréchet   = {float(row['frechet_mm']):.4f} mm{' '*(W-27)}║",
        f"║  起点误差  = {float(row['start_error_mm']):.4f} mm{' '*(W-27)}║",
        f"║  终点误差  = {float(row['end_error_mm']):.4f} mm{' '*(W-27)}║",
        f"║  综合评级  = {str(row['grade']):<{W-15}}║",
        f"╚{bar}╝",
    ]
    return "\n".join(lines)


# ============================================================================
#                         主扫参流程
# ============================================================================

def run_sweep(grids, resume: bool = False):
    """
    执行完整的网格搜索扫参。

    参数：
        grids:  单个参数网格 dict，或多个网格的 list[dict]
                （分别对应 adaptive_spatial / chord_spatial 或两者合并）
        resume: 是否跳过已完成的组合
    """
    ensure_dirs()

    combo_list = build_combo_list(grids)
    total = len(combo_list)

    print(f"\n{'='*60}")
    print(f"  参数扫描启动")
    print(f"  总组合数: {total}")
    print(f"  训练脚本: {os.path.basename(TUNABLE_REG_PY)}")
    print(f"  转换脚本: {os.path.basename(CONVERT_PY)}")
    print(f"  复用脚本: {os.path.basename(APPLY_PY)}")
    print(f"  评估目标: {os.path.basename(TARGET_CSV)}")
    print(f"  结果输出: {SWEEP_CSV}")
    print(f"{'='*60}\n")

    # 检查必要文件
    for path, label in [
        (TUNABLE_REG_PY,  "训练脚本"),
        (CONVERT_PY,      "坐标系转换脚本"),
        (APPLY_PY,        "复用脚本"),
        (TRAJ_ERROR_PY,   "误差评估脚本"),
        (TARGET_CSV,      "目标轨迹CSV"),
    ]:
        if not os.path.exists(path):
            print(f"[ERROR] {label} 不存在: {path}")
            sys.exit(1)

    # 中断恢复：读取已完成的组合
    completed = load_completed_combos(SWEEP_CSV) if resume else set()
    if resume and completed:
        print(f"[恢复模式] 已完成 {len(completed)} 组，跳过重复计算\n")

    # ---- 开始扫参循环 ----
    success_count = 0
    fail_count = 0

    for idx, combo in enumerate(combo_list):
        # 从 combo dict 中提取各参数
        resample_method      = combo["resample_method"]
        loocv                = combo["loocv_candidates"]
        delta                = combo["delta_d"]
        sigma                = combo["gaussian_sigma"]
        noise_level          = combo.get("noise_level")           # adaptive_spatial 専用
        noise_suppression    = combo.get("noise_suppression_factor")  # adaptive_spatial 専用
        densify              = combo.get("densify_factor")         # chord_spatial 専用

        loocv_repr = repr(loocv)
        combo_key  = (
            resample_method,
            loocv_repr,
            str(delta),
            str(sigma),
            str(noise_level),
            str(noise_suppression),
            str(densify),
        )
        combo_id   = idx + 1

        # 跳过已完成
        if resume and combo_key in completed:
            print(f"[{combo_id}/{total}] 跳过（已完成）: loocv={loocv}, delta_d={delta}, sigma={sigma}")
            success_count += 1
            continue

        # 初始化结果行——所有 RESULT_FIELDS 字段必须完整填入
        result_row = {
            "combo_id":               combo_id,
            "resample_method":        resample_method,
            "loocv_candidates":       loocv_repr,
            "delta_d":                "N/A" if delta is None else delta,
            "gaussian_sigma":         "N/A" if sigma is None else sigma,
            "noise_level":            "N/A" if noise_level is None else noise_level,
            "noise_suppression_factor": "N/A" if noise_suppression is None else noise_suppression,
            "densify_factor":         "N/A" if densify is None else densify,
            "apd_mean_mm":            "NaN",
            "apd_max_mm":             "NaN",
            "apd_p95_mm":             "NaN",
            "apd_std_mm":             "NaN",
            "frechet_mm":             "NaN",
            "start_error_mm":         "NaN",
            "end_error_mm":           "NaN",
            "grade":                  "N/A",
            "elapsed_s":              0.0,
            "status":                 "fail",
            "error_msg":              "",
        }

        # 方法专用参数说明字符串
        if resample_method == "adaptive_spatial":
            method_info = f"noise={noise_level}  nsf={noise_suppression}"
        else:
            method_info = f"densify={densify}"

        print(f"\n{'─'*65}")
        print(f"[{combo_id}/{total}]  [{resample_method}]  loocv={loocv}  "
              f"delta_d={delta}mm  sigma={sigma}mm  {method_info}")
        t0 = time.time()

        try:
            # ──────────────────────────────────────────────
            # ① 生成训练脚本临时副本（patch 全部参数）
            # tunable_registration.py 中 TIME_ALIGN["method"] 控制重采样方法。
            # TIME_ALIGN 是文件中最先出现的 "method" 键，count=1 匹配到它。
            # ──────────────────────────────────────────────
            print(f"  [1/4] 训练配准...")
            patches_tunable = {
                "method":           f'"{resample_method}"',  # TIME_ALIGN["method"]
                "loocv_candidates": loocv_repr,
                "delta_d":          str(delta),
                "gaussian_sigma":   str(sigma),
            }
            if resample_method == "adaptive_spatial":
                patches_tunable["noise_level"]              = str(noise_level)
                patches_tunable["noise_suppression_factor"] = str(noise_suppression)
            elif resample_method == "chord_spatial":
                patches_tunable["densify_factor"]           = str(densify)
            patch_py_file(
                src_path = TUNABLE_REG_PY,
                dst_path = TEMP_TUNABLE_PY,
                patches  = patches_tunable,
            )

            rc, out, err = run_script(TEMP_TUNABLE_PY, timeout=TIMEOUT_TRAIN,
                                      cwd=SCRIPT_DIR)
            if rc != 0:
                msg = f"训练失败 (rc={rc}): {(err or out)[-600:]}"
                print(f"  ✗ {msg}")
                result_row["error_msg"] = msg[:500]
                append_result_row(SWEEP_CSV, result_row)
                fail_count += 1
                continue
            print(f"  ✓ 训练完成")

            # ──────────────────────────────────────────────
            # ② 坐标系转换（无参数依赖，直接运行原文件）
            # ──────────────────────────────────────────────
            print(f"  [2/4] 坐标系转换...")
            rc, out, err = run_script(CONVERT_PY, timeout=TIMEOUT_CONVERT,
                                      cwd=SCRIPT_DIR)
            if rc != 0:
                msg = f"坐标系转换失败 (rc={rc}): {(err or out)[-400:]}"
                print(f"  ✗ {msg}")
                result_row["error_msg"] = msg[:500]
                append_result_row(SWEEP_CSV, result_row)
                fail_count += 1
                continue
            print(f"  ✓ 坐标系转换完成")

            # ──────────────────────────────────────────────
            # ③ 生成复用脚本临时副本（保持与训练端一致）
            # apply_transform.py 中 PREPROCESS_CONFIG["resample_method"] 控制重采样方法。
            # ──────────────────────────────────────────────
            print(f"  [3/4] 复用变换...")
            patches_apply = {
                "resample_method": f'"{resample_method}"',  # PREPROCESS_CONFIG["resample_method"]
                "delta_d":         str(delta),
                "gaussian_sigma":  str(sigma),
            }
            if resample_method == "adaptive_spatial":
                patches_apply["noise_level"]              = str(noise_level)
                patches_apply["noise_suppression_factor"] = str(noise_suppression)
            elif resample_method == "chord_spatial":
                patches_apply["densify_factor"]           = str(densify)
            patch_py_file(
                src_path = APPLY_PY,
                dst_path = TEMP_APPLY_PY,
                patches  = patches_apply,
            )
            rc, out, err = run_script(TEMP_APPLY_PY, timeout=TIMEOUT_APPLY,
                                      cwd=SCRIPT_DIR)
            if rc != 0:
                msg = f"复用变换失败 (rc={rc}): {(err or out)[-400:]}"
                print(f"  ✗ {msg}")
                result_row["error_msg"] = msg[:500]
                append_result_row(SWEEP_CSV, result_row)
                fail_count += 1
                continue
            if not os.path.exists(TRANSFORMED_CSV):
                msg = f"复用输出文件不存在: {TRANSFORMED_CSV}"
                print(f"  ✗ {msg}")
                result_row["error_msg"] = msg[:500]
                append_result_row(SWEEP_CSV, result_row)
                fail_count += 1
                continue
            print(f"  ✓ 复用变换完成")

            # ──────────────────────────────────────────────
            # ④ 评估误差（直接 import，关闭可视化）
            # replay = 实际 TCP 录制（TARGET_CSV = tcp6 - tcp.csv）
            # teach  = 变换后的参考轨迹（TRANSFORMED_CSV）
            # ──────────────────────────────────────────────
            print(f"  [4/4] 误差评估...")
            err_data = eval_error(TARGET_CSV, TRANSFORMED_CSV)

            elapsed = time.time() - t0
            result_row.update({
                "apd_mean_mm":   f"{err_data['apd_mean']:.6f}",
                "apd_max_mm":    f"{err_data['apd_max']:.6f}",
                "apd_p95_mm":    f"{err_data['apd_p95']:.6f}",
                "apd_std_mm":    f"{err_data['apd_std']:.6f}",
                "frechet_mm":    f"{err_data['frechet']:.6f}",
                "start_error_mm":f"{err_data['start_error']:.6f}",
                "end_error_mm":  f"{err_data['end_error']:.6f}",
                "grade":         err_data.get("grade", "N/A"),
                "elapsed_s":     f"{elapsed:.1f}",
                "status":        "ok",
                "error_msg":     "",
            })

            print(f"  ✓ APD均值={err_data['apd_mean']:.3f}mm  "
                  f"APD最大={err_data['apd_max']:.3f}mm  "
                  f"Fréchet={err_data['frechet']:.3f}mm  "
                  f"[{elapsed:.0f}s]")
            success_count += 1

        except KeyboardInterrupt:
            print("\n\n[中断] 检测到 Ctrl+C，保存当前进度后退出...")
            result_row["error_msg"] = "用户中断"
            append_result_row(SWEEP_CSV, result_row)
            break
        except Exception as e:
            elapsed = time.time() - t0
            msg = f"{type(e).__name__}: {e}"
            tb = traceback.format_exc()[-800:]
            print(f"  ✗ 异常: {msg}")
            print(f"    {tb}")
            result_row["error_msg"] = msg[:500]
            result_row["elapsed_s"] = f"{elapsed:.1f}"
            fail_count += 1

        # 每组完成后立即写入 CSV（中断恢复保障）
        append_result_row(SWEEP_CSV, result_row)

    # ---- 汇总与最优输出 ----
    print(f"\n{'='*60}")
    print(f"  扫参完成: 成功={success_count}  失败={fail_count}  共={total}")
    print(f"{'='*60}\n")

    summarize_results(SWEEP_CSV)


def summarize_results(csv_path: str):
    """
    读取结果 CSV，按 apd_mean_mm 升序排序，打印汇总表并保存最优 JSON。
    """
    if not os.path.exists(csv_path):
        print(f"[WARN] 结果文件不存在: {csv_path}")
        return

    rows = []
    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    # 只取成功的行
    ok_rows = [r for r in rows if r.get("status") == "ok"]
    if not ok_rows:
        print("[WARN] 没有成功完成的扫参组合，无法生成汇总")
        return

    # 按 apd_mean_mm 升序排序
    try:
        ok_rows.sort(key=lambda r: float(r["apd_mean_mm"]))
    except (ValueError, KeyError):
        pass

    # 打印 Top-15 结果表
    print(f"\n{'─'*120}")
    print(f"{'排名':^4}  {'方法':^16}  {'loocv':^18}  {'delta_d':^7}  {'sigma':^5}  "
          f"{'noise/densify':^20}  {'APD均值mm':^10}  {'APD最大mm':^10}  {'FréchetMM':^10}  {'评级':^12}")
    print(f"{'─'*120}")
    for i, row in enumerate(ok_rows[:15], 1):
        loocv_str = row.get("loocv_candidates", "?")
        if len(loocv_str) > 18:
            loocv_str = loocv_str[:15] + "..."
        method = row.get("resample_method", "?")[:16]
        if row.get("resample_method") == "adaptive_spatial":
            method_params = f"nl={row.get('noise_level','?')} nsf={row.get('noise_suppression_factor','?')}"
        else:
            method_params = f"densify={row.get('densify_factor','?')}"
        if len(method_params) > 20:
            method_params = method_params[:17] + "..."
        print(f"{i:^4}  {method:^16}  {loocv_str:^18}  {row['delta_d']:^7}  {row['gaussian_sigma']:^5}  "
              f"{method_params:^20}  {float(row['apd_mean_mm']):^10.4f}  {float(row['apd_max_mm']):^10.4f}  "
              f"{float(row['frechet_mm']):^10.4f}  {row['grade']:^12}")
    print(f"{'─'*120}")

    # 最优行
    best = ok_rows[0]
    print(f"\n{format_best_box(best)}\n")

    # 保存最优配置 JSON
    def _safe_float(v):
        try: return float(v)
        except: return v
    best_cfg = {
        "resample_method":         best.get("resample_method", "N/A"),
        "loocv_candidates":        best["loocv_candidates"],
        "delta_d":                 _safe_float(best["delta_d"]),
        "gaussian_sigma":          _safe_float(best["gaussian_sigma"]),
        "noise_level":             _safe_float(best.get("noise_level", "N/A")),
        "noise_suppression_factor":_safe_float(best.get("noise_suppression_factor", "N/A")),
        "densify_factor":          _safe_float(best.get("densify_factor", "N/A")),
        "apd_mean_mm":             _safe_float(best["apd_mean_mm"]),
        "apd_max_mm":              _safe_float(best["apd_max_mm"]),
        "apd_p95_mm":              _safe_float(best["apd_p95_mm"]),
        "frechet_mm":              _safe_float(best["frechet_mm"]),
        "start_error_mm":          _safe_float(best["start_error_mm"]),
        "end_error_mm":            _safe_float(best["end_error_mm"]),
        "grade":                   best["grade"],
        "sweep_csv":               csv_path,
    }
    with open(BEST_JSON, "w", encoding="utf-8") as f:
        json.dump(best_cfg, f, indent=2, ensure_ascii=False)
    print(f"最优配置已保存: {BEST_JSON}")
    print(f"完整结果已保存: {csv_path}")


# ============================================================================
#                            CLI 入口
# ============================================================================

def main():
    args = sys.argv[1:]

    if "--help" in args or "-h" in args:
        print(__doc__)
        return

    quick  = "--quick"  in args
    resume = "--resume" in args

    if "--summarize" in args:
        # 仅汇总已有结果
        summarize_results(SWEEP_CSV)
        return

    adaptive = "--adaptive" in args   # 仅扫 adaptive_spatial
    chord    = "--chord"    in args   # 仅扫 chord_spatial

    if quick:
        if adaptive:
            grids = [PARAM_GRID_ADAPTIVE_QUICK]
        elif chord:
            grids = [PARAM_GRID_CHORD_QUICK]
        else:
            grids = PARAM_GRID_QUICK  # 合并两种方法
    else:
        if adaptive:
            grids = [PARAM_GRID_ADAPTIVE_FULL]
        elif chord:
            grids = [PARAM_GRID_CHORD_FULL]
        else:
            grids = PARAM_GRID_FULL  # 合并两种方法

    total_combos = len(build_combo_list(grids))
    mode_desc = []
    if quick:    mode_desc.append("快速验证")
    else:        mode_desc.append("完整扫描")
    if adaptive: mode_desc.append("adaptive_spatial层")
    elif chord:  mode_desc.append("chord_spatial层")
    else:        mode_desc.append("两种方法合并")
    mode = "、".join(mode_desc) + f"（{total_combos}组）"

    print(f"\n运行模式: {mode}")
    if resume:
        print("中断恢复: 开启（跳过已完成组合）")

    run_sweep(grids, resume=resume)


if __name__ == "__main__":
    main()
