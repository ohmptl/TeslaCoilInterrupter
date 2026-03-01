@echo off
cd /d "%~dp0"
echo ========================================
echo  TC-Interrupter Test Suite Setup
echo ========================================

:: Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python not found. Please install Python and try again.
    pause
    exit /b 1
)

:: Create venv if it doesn't already exist
if exist ".venv" (
    echo [INFO] Virtual environment already exists, skipping creation.
) else (
    echo [INFO] Creating virtual environment...
    python -m venv .venv
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to create virtual environment.
        pause
        exit /b 1
    )
    echo [OK] Virtual environment created.
)

:: Activate venv
echo [INFO] Activating virtual environment...
call .venv\Scripts\activate.bat

:: Upgrade pip
echo [INFO] Upgrading pip...
python -m pip install --upgrade pip --quiet

:: Install dependencies
if exist "requirements.txt" (
    echo [INFO] Installing dependencies from requirements.txt...
    pip install -r requirements.txt
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to install dependencies.
        pause
        exit /b 1
    )
    echo [OK] All dependencies installed successfully.
) else (
    echo [ERROR] requirements.txt not found in current directory.
    pause
    exit /b 1
)

echo ========================================
echo  Setup complete! Run your script with:
echo  .venv\Scripts\activate
echo  python your_script.py
echo ========================================
pause
