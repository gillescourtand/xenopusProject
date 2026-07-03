# Xenopus Project installation guide

This document explains how to install and run the Xenopus tracking application after the modular refactor.

The recommended setup on Windows is now:

1. install Python;
2. install the required camera/driver software if needed;
3. create a virtual environment;
4. install `requirements.txt`;
5. run `MotionAnalysis_Xenopus_v2026.py`.

---

## 1. Recommended computer setup

### Operating system

The application is mainly used on **Windows 10 / Windows 11 64-bit**.

It may run on Linux, but the current installation script and camera workflow are written for Windows.

### Hardware

Recommended hardware:

- at least **32 GB RAM** for long recordings or high frame-rate acquisition;
- a recent CPU;
- enough disk space for CSV files and video recordings;
- a Basler camera if live acquisition is required.

### Camera

The live acquisition workflow is designed for Basler cameras, for example the Basler Ace series.

Before installing Python dependencies, install the **Basler pylon software suite** matching the camera and operating system.

The Python package `pypylon` is installed from `requirements.txt`, but it still expects the Basler pylon software/driver to be installed on the computer.

### Optional FTDI trigger

If the hardware trigger is used, install the required FTDI driver first. The Python dependency is `pylibftdi`, but the operating-system driver must also be available.

---

## 2. Project files expected at the root

The project folder should contain at least:

```text
MotionAnalysis_Xenopus_v2026.py
requirements.txt
install_xenopus.bat
xenopus_app/
```

`MotionAnalysis_Xenopus_v2026.py` is the launcher.

`xenopus_app/` contains the modular application code.

`requirements.txt` contains the Python dependencies.

`install_xenopus.bat` creates the virtual environment and installs the dependencies automatically.

---

## 3. Quick installation on Windows

Open the project folder, then double-click:

```text
install_xenopus.bat
```

The script will:

1. check that Python is available;
2. create a local virtual environment named `.venv` if it does not already exist;
3. activate this virtual environment;
4. upgrade `pip`, `setuptools` and `wheel`;
5. install all packages listed in `requirements.txt`;
6. optionally launch the application.

At the end, the application can be started with:

```bat
.venv\Scripts\activate.bat
python MotionAnalysis_Xenopus_v2026.py
```

---

## 4. Manual installation

Use this method if the batch file is not used.

From the project root, open a terminal and run:

```bat
py -3.12 -m venv .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt
python MotionAnalysis_Xenopus_v2026.py
```

If `py -3.12` is not available, use:

```bat
python -m venv .venv
```

---

## 5. Requirements

The main dependencies are installed through `requirements.txt`.

Typical dependencies are:

```text
PyQt5==5.15.11
pyqtgraph==0.14.0
numpy==2.4.6
opencv-python==4.13.0.92
psutil==5.9.7
pypylon==26.5.0
pylibftdi==0.24.0
pygame==2.5.2
keyboard==0.13.5
```

Notes:

- `opencv-python` is used for image processing and tracking;
- `PyQt5` and `pyqtgraph` are used for the graphical interface;
- `pypylon` is used for Basler camera acquisition;
- `pygame` is used for the optokinetic stimulation window;
- `pylibftdi` is used for the optional FTDI trigger;
- `psutil` and `keyboard` are used by legacy video/player utilities.

---

## 6. Running from VS Code or Spyder

### From VS Code

After installation, select the virtual environment interpreter in VS Code:

```text
Ctrl + Shift + P
Python: Select Interpreter
.venv\Scripts\python.exe
```

Then run:

```text
MotionAnalysis_Xenopus_v2026.py
```

If VS Code cannot find `PyQt5`, `cv2`, `pyqtgraph` or `pypylon`, it usually means VS Code is not using the `.venv` interpreter.

### From Spyder

You can also run the application from Spyder.

First, make sure Spyder uses the same Python interpreter as the virtual environment:

```text
.venv\Scripts\python.exe
```

In Spyder, go to:

```text
Tools
Preferences
Python interpreter
Use the following Python interpreter
```

Then select:

```text
.venv\Scripts\python.exe
```

After that, open and run:

```text
MotionAnalysis_Xenopus_v2026.py
```

If Spyder cannot find `PyQt5`, `cv2`, `pyqtgraph` or `pypylon`, it usually means Spyder is not using the `.venv` interpreter.
---

## 7. Common issues

### `ModuleNotFoundError: No module named cv2`

Run:

```bat
.venv\Scripts\activate.bat
python -m pip install -r requirements.txt
```

### `ModuleNotFoundError: No module named PyQt5`

Check that the selected Python interpreter is:

```text
.venv\Scripts\python.exe
```

### `pypylon` cannot access the camera

Check that:

- Basler pylon is installed;
- the camera is connected;
- the camera is detected in pylon Viewer;
- no other software is already using the camera.

### FTDI trigger error

Check that:

- the FTDI device is connected;
- the FTDI driver is installed;
- the device is visible from Windows Device Manager.

---

## 8. Clean reinstall

To reinstall the Python environment from scratch:

1. close the application;
2. delete the `.venv` folder;
3. run `install_xenopus.bat` again.

---

## 9. Start command summary

```bat
.venv\Scripts\activate.bat
python MotionAnalysis_Xenopus_v2026.py
```

The project is ready to run once the dependencies are installed and the required hardware drivers are available.
