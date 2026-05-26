# SafeIntentDetectionPaper

This repository contains the full notebook pipeline for the **Safe Intent Detection** project, developed for multimodal intent recognition in assistive robotics using EEG, EMG, and eye-tracking signals.

The code is organized phase by phase. Users should run the notebooks sequentially because each phase generates files that are required by the next phase.

---

## Project Overview

The goal of this project is to build a safety-aware multimodal intent recognition pipeline for assistive robotic control. The pipeline processes synchronized EEG, EMG, and eye-tracking data, generates action/rest labels, creates subject-independent leave-one-subject-out (LOSO) splits, extracts engineered features, and trains/evaluates a transformer-based model for robust intent detection.

The complete workflow includes:

1. Data cleaning and metadata handling  
2. Schema verification and column standardization  
3. Tri-modal temporal synchronization  
4. HSMM-based action/rest labeling  
5. Label-only export and column validation  
6. Manifest creation and TRUE LOSO split generation  
7. Deterministic signal preprocessing  
8. Fold-wise LOSO data export  
9. Feature extraction  
10. Model training and safety-aware evaluation  

---

## Important Note About Participant Privacy

The original raw data may contain participant-related metadata. To protect participant privacy, the public dataset should be released only after removing personal or identifiable metadata.

Therefore, users can **skip** the following notebook if the released dataset is already anonymized:

```text
Phase1A.ipynb
```

If users start from already cleaned/anonymized files, they should begin from **Phase 1B** or from the earliest phase that matches the released dataset structure.

---

## Recommended Run Order

Run the notebooks in the following order:

```text
1. Phase1A.ipynb
2. Phase1B.ipynb
3. Phase1C.ipynb
4. Phase2A_HSMM.ipynb
5. Phase2B.ipynb
6. Phase3_Manifest.ipynb
7. Phase4_Preprocessing.ipynb
8. Phase5_LOSO.ipynb
9. Phase5_5_Feature_Extraction.ipynb
10. BioRob_Phase6_Main.ipynb
```

Each notebook assumes that the previous phase has already generated the required files.

---

## Notebook Summary

### Phase 1A — Metadata Cleaning and Raw CSV Preparation

**Notebook:** `Phase1A.ipynb`

This phase prepares the original raw CSV files for later processing. It removes unnecessary metadata columns, standardizes core columns, extracts subject/task/trial information from filenames, and prepares cleaned per-trial CSV files.

**Main output:**

```text
Sub-*/cleaned/*.csv
```

**Skip condition:**  
This phase can be skipped if the released dataset is already anonymized and cleaned.

---

### Phase 1B — Schema Verification and Column Standardization

**Notebook:** `Phase1B.ipynb`

This phase scans the cleaned subject folders and verifies that all files follow a consistent training schema. It checks required EEG, EMG, and eye-tracking columns, removes unsupported columns, adds missing expected columns when allowed by the policy, and writes a schema verification report.

**Main outputs:**

```text
train_schema.json
trainschema_prune_report.csv
Sub-*/cleaned/*.csv
```

**Purpose:**  
Ensure that every file has a consistent column format before synchronization.

---

### Phase 1C — Tri-Modal Temporal Synchronization

**Notebook:** `Phase1C.ipynb`

This phase synchronizes EEG, EMG, and eye-tracking streams using timestamps and resamples them onto a common fixed-rate grid. The synchronized files are saved as corrected CSV files for each trial.

**Main outputs:**

```text
Sub-*/cleaned/synchronized_proper_lite_union_v3/*_synchronized_corrected.csv
synchronization_summary.csv
```

**Purpose:**  
Align EEG, EMG, and eye-tracking samples so that each row corresponds to the same time point across modalities.

---

### Phase 2A — HSMM-Based Action/Rest Labeling

**Notebook:** `Phase2A_HSMM.ipynb`

This phase generates weak action/rest labels using a hidden semi-Markov model (HSMM)-style segmentation pipeline. It uses synchronized tri-modal signals to estimate active movement periods and rest periods.

**Main outputs:**

```text
Sub-*/cleaned/synchronized_proper_lite_union_v3/label/*_icml_consensus_labels.csv
Sub-*/cleaned/synchronized_proper_lite_union_v3/label/*_onsets.json
icml_consensus_batch_summary.csv
```

**Important:**  
Eye-tracking validity coding must be consistent across phases. In the cleaned/synchronized files used in this project, eye validity is treated as:

```text
1 = valid / usable
0 = invalid / not worn / unusable
```

Before rerunning this phase on new data, verify the eye-tracking validity convention in the exported files.

---

### Phase 2B — Label-Only Export and Column Guard

**Notebook:** `Phase2B.ipynb`

This phase validates the labeled CSV files and exports the columns required for later model training. It also creates the `labelonly/` folder used by later phases. For non-rest trials, this phase may keep action-dominant labeled rows depending on the configured export policy.

**Main outputs:**

```text
Sub-*/cleaned/synchronized_proper_lite_union_v3/labelonly/*_icml_consensus_labels.csv
column_invalid_or_skipped.csv
active_zero_rows_removed.csv
```

**Purpose:**  
Create compact, schema-consistent labeled files for manifest generation and preprocessing.

---

### Phase 3 — Manifest Creation and TRUE LOSO Splits

**Notebook:** `Phase3_Manifest.ipynb`

This phase builds the dataset manifest and creates subject-independent LOSO train/validation/test splits. It ensures that no subject appears in more than one split within a fold.

**Main outputs:**

```text
_dataset_icml_v1/manifest_v1.csv
_dataset_icml_v1/splits_v1.csv
_dataset_icml_v1/dataset_analysis_report.txt
```

**Purpose:**  
Create leakage-safe subject-level splits before model training.

---

### Phase 4 — Deterministic Signal Preprocessing

**Notebook:** `Phase4_Preprocessing.ipynb`

This phase applies deterministic signal preprocessing to each labeled file. EEG is band-pass filtered, notch filtered when needed, common-average referenced, and artifact-repaired. EMG is converted into an envelope. Eye-tracking features are cleaned using validity/blink indicators, short-gap interpolation, and smoothing.

**Main outputs:**

```text
*.preproc.npz
*.preproc_log.json
_dataset_icml_v1/qc_summary_v1.csv
_dataset_icml_v1/qc_summary_by_subject_v1.csv
```

**Important signal-shape convention:**

```text
EEG: time × channels
EMG: time × channels
ET : time × features
```

Filtering should be applied along the time axis.

---

### Phase 5 — LOSO Exporter

**Notebook:** `Phase5_LOSO.ipynb`

This phase converts the preprocessed Phase 4 caches into fold-wise training, validation, and test shards. It computes train-only normalization statistics and exports fixed-length windows for supervised training and self-supervised learning.

**Main outputs:**

```text
_dataset_icml_v1/exports_v1_balanced_fold*/
_dataset_icml_v1/exports_v1_ssl_fold*/
```

**Notes:**

- Balanced exports are used for supervised training.
- SSL exports are used for self-supervised pretraining.
- Masks are used internally for coverage filtering and normalization.
- The final model shards contain normalized input tensors (`X_EEG`, `X_EMG`, `X_ET`) and labels. They do not require `M_EEG`, `M_EMG`, `M_ET`, or `ET_valid` as direct model inputs.

---

### Phase 5.5 — EEG/EMG Feature Extraction

**Notebook:** `Phase5_5_Feature_Extraction.ipynb`

This phase extracts engineered window-level features from the Phase 5 supervised exports. EEG features include PSD/bandpower and Hjorth-style descriptors. EMG features include time-domain envelope features such as RMS, MAV, waveform length, and variance-related descriptors.

**Main outputs:**

```text
_dataset_icml_v1/features_v1_eeg_psd_full_fold{fold}_{split}.npz
```

**Purpose:**  
Provide compact engineered EEG/EMG features that can be fused with the deep model representation in Phase 6.

---

### Phase 6 — SSL Pretraining, Supervised Fine-Tuning, and Safety Evaluation

**Notebook:** `BioRob_Phase6_Main.ipynb`

This phase trains the TriSaFe-Trans model. It first performs fold-specific self-supervised pretraining using SSL exports, then fine-tunes the model using balanced supervised LOSO exports. It evaluates action/task decoding, modality ablations, sensor-dropout scenarios, and safety policies.

**Main outputs:**

```text
saved model checkpoints
fold-level evaluation files
summary metrics
safety-policy results
robustness results
```

**Evaluated scenarios:**

```text
S0 = all sensors available
S1 = EEG dropped
S2 = EMG dropped
S3 = eye-tracking dropped
```

**Safety policies:**

```text
P0 = raw action probability
P1 = gate-/availability-based reliability proxy
P2 = reliability plus uncertainty penalty
```

---

## Expected Directory Structure

A typical project directory should look like this:

```text
SafeIntentDetectionPaper/
├── Phase1A.ipynb
├── Phase1B.ipynb
├── Phase1C.ipynb
├── Phase2A_HSMM.ipynb
├── Phase2B.ipynb
├── Phase3_Manifest.ipynb
├── Phase4_Preprocessing.ipynb
├── Phase5_LOSO.ipynb
├── Phase5_5_Feature_Extraction.ipynb
├── BioRob_Phase6_Main.ipynb
├── README.md
└── data/
```

The actual data directory may be outside the repository. Update the root path variables inside each notebook before running.

---

## Running the Pipeline

### Step 1 — Update paths

Before running any notebook, update the root directory path in the configuration cell. Example:

```python
ROOT_DIR = Path("/path/to/project/data")
```

All later phases depend on this path.

---

### Step 2 — Run notebooks sequentially

Run each notebook from top to bottom in the order listed above.

Do not run Phase 3, Phase 4, Phase 5, or Phase 6 before the previous phase outputs exist.

---

### Step 3 — Check generated outputs

After each phase, confirm that the expected output folder or file exists. For example:

```text
Phase 3 should create manifest_v1.csv and splits_v1.csv
Phase 4 should create .preproc.npz files
Phase 5 should create fold-wise NPZ shards
Phase 6 should create model/evaluation outputs
```

---

## Reproducibility and Safety Checks

Before reporting results, verify the following:

- EEG/EMG/ET arrays follow the expected `time × channels/features` convention.
- Filtering is applied along the time axis.
- Train/validation/test splits are subject-independent.
- Normalization statistics are computed from training data only.
- Validation is used for threshold selection; test data is not used for tuning.
- Eye-tracking validity coding is consistent across phases.
- The final number of subjects/files/windows is reported from `manifest_v1.csv` and `splits_v1.csv`, not from raw data counts.
- Phase 5 shards match the Phase 6 model input requirements.

---

## Notes on Model Inputs

The Phase 6 model uses the following required arrays from Phase 5 shards:

```text
X_EEG
X_EMG
X_ET
y_action
y_task
```

The model does not require direct mask-array inputs such as:

```text
M_EEG
M_EMG
M_ET
ET_valid
```

Signal-quality masks are used during preprocessing/export for coverage filtering and normalization, but they are not direct model inputs in the current Phase 6 implementation.

---

## Citation

If you use this code or dataset, please cite the related paper:

```bibtex
@inproceedings{-----I will update it later,
  title     = {TriSaFe-Trans: A Safety-Aware Multimodal Intent Recognition Pipeline for Assistive Robotics},
  author    = {Sultan, Tipu and Cool, Kody and Liu, Guangping and Tamilselvan, Gajapriya and Babaiasl, Madi},
  booktitle = {Proceedings of the IEEE RAS/EMBS International Conference on Biomedical Robotics and Biomechatronics},
  year      = {2026}
}
```

---

## Contact

For questions, please contact the corresponding authors listed in the paper or open an issue in this repository.
