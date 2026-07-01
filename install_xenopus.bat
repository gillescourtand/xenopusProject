@echo off
setlocal EnableExtensions

cd /d "%~dp0"

echo ============================================================
echo Xenopus Project - installation helper
echo ============================================================
echo.

set "LAUNCHER=MotionAnalysis_Xenopus_v2026.py"
set "VENV_DIR=.venv"
set "REQ_FILE=requirements.txt"

if not exist "%LAUNCHER%" (
    echo [ERROR] %LAUNCHER% was not found in this folder.
    echo Please place this .bat file at the project root.
    echo.
    pause
    exit /b 1
)

if not exist "%REQ_FILE%" (
    echo [ERROR] %REQ_FILE% was not found in this folder.
    echo Please add requirements.txt at the project root.
    echo.
    pause
    exit /b 1
)

echo [1/5] Looking for Python...
set "PYTHON_CMD=py -3.12"
%PYTHON_CMD% --version >nul 2>nul
if errorlevel 1 (
    set "PYTHON_CMD=python"
)

%PYTHON_CMD% --version >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python was not found.
    echo Install Python 3.12 64-bit, then run this script again.
    echo.
    pause
    exit /b 1
)

%PYTHON_CMD% --version
echo.

echo [2/5] Creating virtual environment if needed...
if not exist "%VENV_DIR%\Scripts\python.exe" (
    %PYTHON_CMD% -m venv "%VENV_DIR%"
    if errorlevel 1 (
        echo [ERROR] Could not create the virtual environment.
        pause
        exit /b 1
    )
) else (
    echo Virtual environment already exists: %VENV_DIR%
)

echo.
echo [3/5] Activating virtual environment...
call "%VENV_DIR%\Scripts\activate.bat"
if errorlevel 1 (
    echo [ERROR] Could not activate the virtual environment.
    pause
    exit /b 1
)

echo.
echo [4/5] Upgrading pip tools...
python -m pip install --upgrade pip setuptools wheel
if errorlevel 1 (
    echo [ERROR] pip upgrade failed.
    pause
    exit /b 1
)

echo.
echo [5/5] Installing project requirements...
python -m pip install -r "%REQ_FILE%"
if errorlevel 1 (
    echo.
    echo [ERROR] Requirements installation failed.
    echo If the error concerns pypylon, install Basler pylon first, then retry.
    echo If the error concerns pylibftdi, check the FTDI driver installation.
    echo.
    pause
    exit /b 1
)

echo.
echo ============================================================
echo Installation completed.
echo ============================================================
echo.
echo Reminder:
echo - Basler camera support requires Basler pylon installed on Windows.
echo - FTDI trigger support requires the FTDI driver installed on Windows.
echo.

set /p RUN_APP="Start the Xenopus application now? [Y/N]: "
if /I "%RUN_APP%"=="Y" (
    echo.
    echo Starting %LAUNCHER% ...
    python "%LAUNCHER%"
) else (
    echo.
    echo To start later:
    echo call %VENV_DIR%\Scripts\activate.bat
    echo python %LAUNCHER%
)

echo.
pause
endlocal
