@echo off
title Traffic Semantic Intersection Scenario Selector

echo ========================================
echo   Traffic Semantic Intersection Scenario Selector
echo ========================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Python not detected. Please install Python 3.9.0-3.11.0.
    echo Download: https://www.python.org/downloads/
    echo.
    pause
    exit /b 1
)

echo Starting Traffic Semantic Intersection Scenario Selector...
echo.

REM Start Traffic Semantic Intersection Scenario Selector
python Transportation_Semantic_Selector.py

if %errorlevel% neq 0 (
    echo.
    echo Error: Failed to start Traffic Semantic Intersection Scenario Selector.
    echo Please make sure you have installed all dependencies.
    echo.
    pause
    exit /b 1
)

echo Traffic Semantic Intersection Scenario Selector started.   
pause