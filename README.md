# VR Lighthouse-Based Rapid Teaching System for Collaborative Robots with Execution-Feedback Trajectory Error Compensation

[![Journal](https://img.shields.io/badge/Journal-Robotics%20and%20Computer--Integrated%20Manufacturing-blue)](https://www.sciencedirect.com/journal/robotics-and-computer-integrated-manufacturing)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

## Demo Video

演示视频展示系统的快速示教与回放流程（含外部跟踪与轨迹校正的关键环节）：

- [5月20日.mp4](5%E6%9C%8820%E6%97%A5.mp4)

## Paper Information

**Title:** VR Lighthouse-Based Rapid Teaching System for Collaborative Robots with Execution-Feedback Trajectory Error Compensation

**Authors:** WenHao Wang, Xiaolong Xu†, Yujie Sun†, Yong Song, Lelai Zhou, Xincheng Tian

> † Corresponding authors: xuxiaolong@sdu.edu.cn, sunyujie@sdu.edu.cn

**Affiliations:**
- School of Airspace Science and Engineering, Shandong University, Weihai 264209, China
- Shandong Key Laboratory of Intelligent Electronic Packaging Testing and Application, Weihai 264209, China
- School of Control Science and Engineering, Shandong University, Jinan 250100, China

**Journal:** *Robotics and Computer-Integrated Manufacturing* (Elsevier, RCIM)

---

## Abstract

Existing programming-by-demonstration (PbD) systems with external tracking typically use the tracker only for teaching-input acquisition and rely on the robot controller for execution feedback. This places the demonstrated and executed trajectories in different reference frames, making trajectory-level error analysis and compensation difficult.

This paper presents a rapid teaching system for collaborative robots using **VR lighthouse** (HTC Vive optical tracking) for external localization. It acquires intuitive hand-guided trajectories through a handheld teaching device and establishes a unified coordinate chain through tip and hand-eye calibrations. A tracker rigidly mounted on the robot end-effector records both demonstrated and executed trajectories in the **same lighthouse reference frame**.

Based on this unified reference, a **trajectory-conditioned replay-correction workflow** is proposed. The workflow employs:
- Chord-length resampling (time-uniform → spatially uniform samples)
- Normalized arc-length parameterization (geometry-based trajectory correspondence)
- Moving Least Squares (MLS)-based local compensation field (spatially non-uniform error modeling)

Experiments on four representative trajectory types, validated by an independent NOKOV motion capture system, show:
- **Mean APD < 0.65 mm** for all trajectory types after correction
- **Mean Fréchet distance < 1.52 mm** for all trajectory types after correction
- For the complex workpiece-contour task, mean APD decreased by **79.1%** relative to the global rigid correction (SVD/Kabsch) baseline

---

## Repository Structure

```
.
├── Collaborative Robots Teach-in System_unity/   # Unity software project
   ├── Assets/
   │   ├── Scripts/          # Core C# scripts (calibration, trajectory recording, TCP communication)
   │   ├── HTC.UnityPlugin/  # HTC Vive SteamVR integration
   │   ├── Nokov/            # NOKOV motion capture system interface
   │   ├── Scenes/           # Unity scene files
   │   └── ...
   ├── Packages/
   └── ProjectSettings/

```
---

## Unity Software

### Overview

The Unity project (`Collaborative Robots Teach-in System_unity/`) implements the complete **Collaborative Robots Teach-in System** described in the paper. It serves as the central data processing hub, managing:

- Real-time tracker pose acquisition from HTC Vive via SteamVR
- Multi-source data synchronization (demonstration tracker, end-effector tracker, robot TCP)
- TCP/IP communication with the UR5 robot controller
- Trajectory recording, buffering, and export
- Calibration procedures (probe tip calibration and hand-eye calibration)

### System Requirements

| Component | Specification |
|-----------|--------------|
| Unity Version | 2020.3 LTS or later |
| Operating System | Windows 10/11 |
| SteamVR | Latest version (via Steam) |
| HTC Vive Trackers | 2× Vive Tracker (3.0 recommended) |
| Lighthouse Base Stations | 2× SteamVR base stations |
| Robot | Universal Robots UR5 (UR software 5.x, with network access) |
| NOKOV SDK | Optional – only required for NOKOV motion capture integration |

### Key Features

1. **Teaching Acquisition Module**
   - Records tracker pose sequence in real time from the handheld pen-shaped teaching device
   - Applies probe tip offset (from tip calibration) to recover the true TCP path

2. **Calibration Module**
   - *Probe tip calibration:* Pivot calibration to determine the fixed spatial offset between the tracker body and the pen tip
   - *Hand-eye calibration:* Tsai AX=XB formulation to establish the rigid mapping from the lighthouse frame to the UR5 robot base frame

3. **Execution and Feedback Module**
   - Converts the demonstrated trajectory to robot base frame and generates UR5 motion commands
   - Simultaneously records the end-effector tracker pose (execution feedback) and the robot TCP state during playback

4. **Unified External Reference**
   - Both demonstration trajectory and execution-feedback trajectory are recorded in the same lighthouse coordinate frame, enabling direct comparison and error field modeling

### How to Use

1. Clone this repository and open the `Collaborative Robots Teach-in System_unity/` folder as a Unity project.
2. Ensure SteamVR is running and both HTC Vive base stations and trackers are detected.
3. Connect the Windows host PC to the UR5 robot controller via LAN (configure the robot IP in the Unity Inspector).
4. Open the main scene in `Assets/Scenes/`.
5. Follow the in-software calibration workflow:
   - **Step 1 – Probe tip calibration:** Press the teaching device tip against a fixed point and rotate; confirm when sufficient poses are collected.
   - **Step 2 – Hand-eye calibration:** Drive the UR5 to a set of spatially distributed poses while recording synchronized tracker and TCP poses; solve for `T_B_L`.
6. Perform the demonstration trajectory with the handheld device.
7. Initiate open-loop playback; the system records `D` (demo), `F` (execution feedback), and `P` (controller TCP) concurrently.
8. Run the replay-correction workflow (offline post-processing) to generate the corrected trajectory `P_corr` for re-execution.

---

## Citation

If you use this software or data in your research, please cite our paper:

```bibtex
@article{wang2025vr,
  title   = {VR Lighthouse-Based Rapid Teaching System for Collaborative Robots
             with Execution-Feedback Trajectory Error Compensation},
  author  = {Wang, WenHao and Xu, Xiaolong and Sun, Yujie and Song, Yong
             and Zhou, Lelai and Tian, Xincheng},
  journal = {Robotics and Computer-Integrated Manufacturing},
  year    = {2025},
  publisher = {Elsevier}
}
```

> **Note:** Please update the DOI and volume/page information once the paper is formally published.

---

## License

This project is released under the [MIT License](LICENSE).

The experimental data (`data.csv/`) is released under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

---

## Contact

School of Airspace Science and Engineering, Shandong University, Weihai, China
