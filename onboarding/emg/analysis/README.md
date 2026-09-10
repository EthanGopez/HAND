# Beginner EMG Analysis

`beginner_emg_analysis.ipynb` is a guided, hardware-free introduction to the
software behind the Phase 1 EMG intent detector. It loads the synthetic analog
envelope, plots and smooths it, estimates a resting baseline, normalizes the
signal, selects thresholds from calibration windows, applies hysteresis and
debounce, and measures false activations.

From the repository root:

```bash
python3 -m venv .venv-emg
source .venv-emg/bin/activate
python -m pip install --upgrade pip
python -m pip install -r onboarding/emg/analysis/requirements.txt
jupyter lab onboarding/emg/analysis/beginner_emg_analysis.ipynb
```

On Windows PowerShell, activate with:

```powershell
.\.venv-emg\Scripts\Activate.ps1
```

Use Python 3.10 or 3.11. Run the notebook from top to bottom and answer the
reflection questions in its final section.
