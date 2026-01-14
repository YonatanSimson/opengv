@echo off
REM Quick test script to verify Windows build works
REM Usage: scripts\test_windows_build.bat

setlocal enabledelayedexpansion

cd /d "%~dp0\.."

echo Testing OpenGV Windows Build
echo =============================

REM Activate conda environment
call conda activate opengv
if %errorlevel% neq 0 (
    echo ERROR: Failed to activate conda environment
    exit /b 1
)

REM Clean previous build
if exist build rmdir /s /q build
mkdir build
cd build

echo.
echo Step 1: Configuring CMake...
cmake .. -DBUILD_PYTHON=ON -DBUILD_TESTS=ON -A x64 -DCMAKE_CXX_FLAGS="/MP /EHsc" 
if %errorlevel% neq 0 (
    echo ERROR: CMake configuration failed
    exit /b 1
)
echo [OK] CMake configured successfully

echo.
echo Step 2: Building (Release, parallel)...
cmake --build . --config Release --parallel
if %errorlevel% neq 0 (
    echo ERROR: Build failed
    exit /b 1
)
echo [OK] Build completed successfully

echo.
echo Step 3: Running C++ tests...
ctest -C Release --output-on-failure
if %errorlevel% neq 0 (
    echo [WARN] Some C++ tests failed
) else (
    echo [OK] All C++ tests passed
)

echo.
echo Step 4: Checking outputs...
if exist lib\Release\opengv.lib (
    echo [OK] Found: lib\Release\opengv.lib
) else (
    echo [WARN] opengv.lib not found in lib\Release
)

if exist lib\Release\pyopengv.pyd (
    echo [OK] Found: lib\Release\pyopengv.pyd
) else (
    echo [WARN] pyopengv.pyd not found in lib\Release
)

echo.
echo Step 5: Testing Python module...
cd ..
set PYTHONPATH=%CD%\build\lib\Release
pip install pytest >nul 2>&1
pytest python\ -v
if %errorlevel% neq 0 (
    echo [WARN] Some Python tests failed
) else (
    echo [OK] All Python tests passed
)
cd build

echo.
echo =============================
echo Windows build test completed!
echo =============================

endlocal
