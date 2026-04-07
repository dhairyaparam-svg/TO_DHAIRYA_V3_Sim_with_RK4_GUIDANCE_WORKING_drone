@echo off
echo ========================================================
echo Launching Quadrotor SMC-IO Flight Guidance Simulator...
echo ========================================================
echo.
echo Starting Streamlit web server...
python -m streamlit run quadrotor_web_launcher.py
pause
