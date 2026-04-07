@echo off
REM Setup Script for SMC-IO Web Launcher
REM This script installs all required dependencies

echo.
echo ======================================================
echo  SMC-IO Asteroid Landing Guidance - Setup
echo ======================================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not in PATH
    echo Please install Python from https://www.python.org
    pause
    exit /b 1
)

echo [OK] Python is installed
echo.
echo Installing required packages...
echo ======================================================
echo.

REM Install Streamlit
echo Installing Streamlit...
pip install streamlit
if errorlevel 1 (
    echo [ERROR] Failed to install Streamlit
    pause
    exit /b 1
)
echo [OK] Streamlit installed
echo.

REM Install Plotly
echo Installing Plotly...
pip install plotly
if errorlevel 1 (
    echo [WARNING] Failed to install Plotly (may already exist)
)
echo.

REM Install other packages
echo Installing NumPy, SciPy, Matplotlib...
pip install numpy scipy matplotlib
if errorlevel 1 (
    echo [WARNING] Some packages failed to install
)
echo.

echo ======================================================
echo  Setup Complete!
echo ======================================================
echo.
echo Launching web interface in 2 seconds...
echo This will open in your browser at:
echo   http://localhost:8501
echo.
timeout /t 2 >nul

REM Launch Streamlit
echo Starting Streamlit...
streamlit run web_launcher.py
if errorlevel 1 (
    echo.
    echo [ERROR] Failed to launch Streamlit
    echo Please try manually running:
    echo   streamlit run web_launcher.py
    echo.
    pause
    exit /b 1
)

pause
