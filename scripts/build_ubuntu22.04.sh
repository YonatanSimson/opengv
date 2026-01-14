#!/bin/bash
# Build script for Ubuntu 22.04 - mirrors the GitHub Actions CI workflow
# Usage: ./scripts/build_ubuntu22.04.sh
#
# This script performs the exact same steps as the build-and-test-ubuntu workflow

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "========================================"
echo "OpenGV Ubuntu 22.04 Build"
echo "========================================"
echo ""

# Step 1: Install dependencies
echo "Step 1: Installing system dependencies..."
echo "----------------------------------------"
sudo apt-get update
sudo apt-get install -y \
    cmake \
    build-essential \
    libeigen3-dev \
    python3-dev \
    python3-pip \
    python3-venv \
    pybind11-dev
echo "[OK] System dependencies installed"
echo ""

# Step 2: Configure CMake (with Python and tests)
echo "Step 2: Configuring CMake..."
echo "----------------------------"
rm -rf build
mkdir -p build
cd build
cmake .. \
    -DBUILD_TESTS=ON \
    -DBUILD_PYTHON=ON \
    -DPYTHON_EXECUTABLE=$(which python3)
echo "[OK] CMake configured"
echo ""

# Step 3: Build everything (C++ and Python)
echo "Step 3: Building (C++ and Python)..."
echo "-------------------------------------"
make -j$(nproc)
echo "[OK] Build completed"
echo ""

# Step 4: Run C++ tests
echo "Step 4: Running C++ tests..."
echo "----------------------------"
ctest --output-on-failure
echo "[OK] C++ tests completed"
echo ""

# Step 5: Test Python module
echo "Step 5: Testing Python module..."
echo "---------------------------------"
cd ..
python3 -m venv .venv
source .venv/bin/activate
pip install numpy
PYTHONPATH=build/lib python3 python/run_all_tests.py
echo "[OK] Python tests completed"
echo ""

echo "========================================"
echo "Ubuntu 22.04 build completed successfully!"
echo "========================================"
