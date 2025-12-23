# SafeIntentDetectionPaperDec2025  
**Companion repository** for the paper *Safety-Aware Multimodal Intent Recognition with EEG, EMG, and Eye Tracking for Assistive Robotics (TriSaFe-Trans)*.

This repository provides two deliverables:

1) **TriSaFe-Trans training/evaluation pipeline** (Phase 1–6) for safety-aware multimodal intent recognition from **EEG + EMG + Eye Tracking**.  
2) **Real-robot implementation** on **Kinova Gen3 (6-DoF) + Robotiq 2F-85** using **ROS 2**, where an inferred **task ID** triggers autonomous execution by replaying **pre-recorded joint waypoint trajectories**.

> **Status:** Code release for reproducibility. **Dataset will be made available later** (see [Data availability](#data-availability)).

---

## Table of contents
- [What this repository contains](#what-this-repository-contains)
- [Repository layout](#repository-layout)
- [Quickstart](#quickstart)
  - [A) ML pipeline](#a-ml-pipeline)
  - [B) Real robot (ROS 2)](#b-real-robot-ros-2)
- [TriSaFe-Trans pipeline (Phase 1–6)](#trisafe-trans-pipeline-phase-16)
- [Real robot implementation (Kinova Gen3 + Robotiq 2F-85)](#real-robot-implementation-kinova-gen3--robotiq-2f-85)
- [Figures](#figures)
- [Data availability](#data-availability)
- [Citation](#citation)
- [License](#license)

---

## What this repository contains

### 1) TriSaFe-Trans ML pipeline (Implementation/ + Notebooks/)
The ML code is organized in two places:

**Implementation/**  
- Contains the **primary run entry** for training/evaluation used in this repository:
  - `Phase-6(All-Sub).ipynb` — train/evaluate in an “all-subject” configuration
  - `Download_train_Model.text` — instructions/notes for downloading a trained checkpoint

**Notebooks/**  
- Contains the **full Phase 1–6 pipeline notebooks** used to reproduce the BioRob experiments:
  - Phase 1: metadata + scanning + synchronization
  - Phase 2: safety-aware labeling (HSMM-like)
  - Phase 3: manifest + TRUE LOSO splits
  - Phase 4: deterministic preprocessing/caching
  - Phase 5: LOSO exporter
  - Phase 5.5: feature extraction
  - Phase 6: training + robustness/policy evaluation

> If you are here for the model and experiments, start with **Implementation/** for a fast run, or **Notebooks/** for full reproduction.

### 2) Real robot execution (kinova-joint-tasks-main/)
The **kinova-joint-tasks-main/** folder contains **all ROS 2 code for real-robot implementation**, including:
- pre-recorded waypoint task execution on **Kinova Gen3 (6-DoF)**,
- gripper control for **Robotiq 2F-85**,
- an execution wrapper that can **run tasks manually** or **trigger tasks from TriSaFe-Trans inference** (human sensor → task ID → trajectory replay).

> If you are here to run the robot (ROS 2 + Kinova), start in **kinova-joint-tasks-main/**.

### 3) Notebooks and figures
- **Notebooks/**: full pipeline notebooks, plots, and reporting utilities.
- **figures/**: figures used in the paper + this README.

---

## Repository layout

```text
SafeIntentDetectionPaperDec2025/
├── Implementation/                 # Minimal entry for training/inference (all-subject run)
│   ├── Phase-6(All-Sub).ipynb      # Main notebook: train/evaluate (all subjects)
│   └── Download_train_Model.text   # Notes to download trained model/checkpoints
│
├── Notebooks/                      # Full Phase 1–6 pipeline notebooks (reproducibility)
│   ├── Phase-1-A(Meta-Data).ipynb
│   ├── Phase-1-B(Scans each Sub).ipynb
│   ├── Phase-1-C-Synchronizer.ipynb
│   ├── Phase-2-A- Labeler — HSMM (Single-Action).ipynb
│   ├── Phase-2-B.ipynb
│   ├── Phase 3 — Manifest & TRUE LOSO splits.ipynb
│   ├── Phase 4 — Deterministic Preprocessing .ipynb
│   ├── Phase 5 — LOSO Exporter.ipynb
│   ├── Phase 5.5 — Feature Extraction.ipynb
│   └── Phase 6 (BioRob RQ) .ipynb
│
├── figures/                        # Paper/README figures
├── kinova-joint-tasks-main/        # ROS 2 + Kinova Gen3 + Robotiq execution (waypoints + inference trigger)
└── README.md


## Figures

### Pipeline overview (Phase 1–6)
![Pipeline overview](figures/Systemoverview.png)

### System overview
![System overview](figures/Systemoverview.png)

### Experimental protocol and labeling timeline
![Protocol](figures/protocol.png)

### TriSaFe-Trans architecture
![Architecture](figures/Transarchitecture.png)

---

## Environment setup

### Recommended
- Python **3.10+**
- OS: Ubuntu 20.04/22.04 recommended (works on Windows with path adjustments)
- GPU optional (Phase 6 training benefits from CUDA)

### Install dependencies
Create a clean environment:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install numpy pandas scipy scikit-learn matplotlib tqdm jupyter
# If you run Phase-6 training:
pip install torch


## We will update this part
@inproceedings{-----------,
  title        = {Safety-Aware Multimodal Intent Recognition with EEG, EMG, and Eye Tracking for Assistive Robotics},
  author       = {Tipu Sultan and Md Shariful Islam and others},
  booktitle    = {IEEE/RAS-EMBS International Conference on Biomedical Robotics and Biomechatronics (BioRob)},
  year         = {2025},
  note         = {To appear. Update this entry with DOI/URL when available.}
}
