# SafeIntentDetectionPaper

This repository contains the full notebook pipeline for the **Safe Intent Detection** project, developed for multimodal intent recognition in assistive robotics using EEG, EMG, and eye-tracking signals.

The code is organized phase by phase. Users should run the notebooks sequentially because each phase generates files that are required by the next phase.

---

## Project Overview

The goal of this project is to build a safety-aware multimodal intent recognition pipeline for assistive robotic control. The pipeline processes synchronized EEG, EMG, and eye-tracking data, generates action labels, creates LOSO subject-independent splits, extracts features, and trains/evaluates a transformer-based model for robust intent detection.

The complete workflow includes:

1. Data cleaning and metadata handling  
2. Signal synchronization  
3. HSMM-based action/rest labeling  
4. Manifest creation and TRUE LOSO split generation  
5. Deterministic preprocessing  
6. Fold-wise LOSO data export  
7. Feature extraction  
8. Model training and safety-aware evaluation  

---

## Important Note About Participant Privacy

The original raw data may contain participant-related metadata.  
To protect participant privacy, the public dataset will be uploaded after removing personal or identifiable metadata.

Therefore, users can **skip** the following notebook if the released dataset is already anonymized:

```text
Phase-1-A(Meta-Data).ipynb
