# SafeIntentDetectionPaperDec2025  
**Companion repository** for the paper *Safety-Aware Multimodal Intent Recognition with EEG, EMG, and Eye Tracking for Assistive Robotics (TriSaFe-Trans)*.  
This repo provides: **(1) full training/evaluation code** (Implementation/) and
**(2) real-robot execution code** for **Kinova Gen3 + Robotiq 2F-85** via **ROS 2** (kinova-joint-tasks-main/), along with figures and supporting notebooks.

> **Status:** Code release for reproducibility. **Dataset will be made available later** (see [Data availability](#data-availability)).

---

## What this repository contains

### 1) TriSaFe-Trans ML pipeline (Implementation/)
The **Implementation/** folder contains **all training and evaluation code** for the BioRob paper pipeline, including:
- multimodal synchronization (EEG/EMG/ET),
- labeling (REST vs ACTION + task),
- LOSO split generation,
- deterministic preprocessing/caching,
- SSL pretraining + supervised fine-tuning,
- robustness evaluation (S0–S3) and policy-level evaluation (P0–P2) using safety metrics (SAE/UAR/MIR/NRA).

> If you are here for the ML model and experiments, start in **Implementation/**.

### 2) Real robot execution (kinova-joint-tasks-main/)
The **kinova-joint-tasks-main/** folder contains **all ROS 2 code for real-robot implementation**, including:
- pre-recorded waypoint task execution on **Kinova Gen3 (6-DoF)**,
- gripper control for **Robotiq 2F-85**,
- an execution wrapper that can **run tasks manually** or **trigger tasks from TriSaFe-Trans inference** (human sensor → task ID → trajectory replay).

> If you are here to run the robot (ROS 2 + Kinova), start in **kinova-joint-tasks-main/**.

### 3) Notebooks and figures
- **Notebooks/**: analysis notebooks, plots, and reporting utilities.
- **figures/**: figures used in the paper + this README.

---

## Figures

### Pipeline overview (Phase 1–6)
![Pipeline overview](figures/awarepipeline.png)

### System overview
![System overview](figures/Systemoverview.png)

### Experimental protocol and labeling timeline
![Protocol](figures/protocol.png)

### TriSaFe-Trans architecture
![Architecture](figures/Transarchitecture.png)

---

## Repository layout

```text
SafeIntentDetectionPaperDec2025/
├── Implementation/              # ✅ All model training + preprocessing + evaluation code (Phase 1–6)
├── Notebooks/                   # Analysis notebooks, plots, reporting
├── figures/                     # Paper/README figures
├── kinova-joint-tasks-main/     # ✅ ROS 2 + Kinova Gen3 real-robot execution (waypoints + inference trigger)
└── README.md
