# Python Wheel Building and Distribution Guide

This document describes how to build, install, and distribute PyOpenGV as a Python wheel package.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Installation Options](#installation-options)
3. [Building Wheels](#building-wheels)
4. [Publishing to PyPI](#publishing-to-pypi)
5. [Troubleshooting](#troubleshooting)

## Quick Start

### For Users

Install from PyPI (when published):
```bash
pip install pyopengv
```

### For Developers

1. Clone the repository
2. Install build dependencies:
   ```bash
   ./build_wheel.sh install-deps
   ```
3. Install in development mode:
   ```bash
   pip install -e .
   ```

## Installation Options

### Option 1: Install from PyPI (Recommended for users)

```bash
pip install pyopengv
```

### Option 2: Install from Wheel File

```bash
# Build the wheel
./build_wheel.sh build

# Install from built wheel
./build_wheel.sh install-wheel
```

Or manually:
```bash
pip install build
python -m build
pip install dist/pyopengv-*.whl
```

### Option 3: Install in Development Mode

For active development:
```bash
pip install -e .
```

This creates a symbolic link to your source code, so changes are immediately reflected.

### Option 4: Install from Source

```bash
pip install .
```

## Building Wheels

### Using the Helper Script (Recommended)

The `build_wheel.sh` script provides a convenient interface:

```bash
# Install build dependencies
./build_wheel.sh install-deps

# Build wheel
./build_wheel.sh build

# Install from wheel
./build_wheel.sh install-wheel

# Clean build artifacts
./build_wheel.sh clean

# Run tests
./build_wheel.sh test
```

### Manual Build Process

1. Install build tools:
   ```bash
   pip install build wheel setuptools
   ```

2. Build the wheel:
   ```bash
   python -m build
   ```

3. The wheel will be created in `dist/`:
   ```bash
   ls dist/
   # Output: pyopengv-1.0.0-cp39-cp39-linux_x86_64.whl
   ```

4. Install the wheel:
   ```bash
   pip install dist/pyopengv-*.whl
   ```

## Publishing to PyPI

### Prerequisites

1. Create accounts on PyPI and TestPyPI:
   - PyPI: https://pypi.org/account/register/
   - TestPyPI: https://test.pypi.org/account/register/

2. Generate API tokens:
   - PyPI token: https://pypi.org/manage/account/token/
   - TestPyPI token: https://test.pypi.org/manage/account/token/

3. Configure credentials (copy .pypirc.template to ~/.pypirc):
   ```bash
   cp .pypirc.template ~/.pypirc
   # Edit ~/.pypirc and add your tokens
   chmod 600 ~/.pypirc
   ```

### Upload to TestPyPI (Recommended First)

Test your package on TestPyPI before publishing to production:

```bash
# Using helper script
./build_wheel.sh upload-test

# Or manually
pip install twine
python -m build
twine upload --repository testpypi dist/*
```

Test installation from TestPyPI:
```bash
pip install --index-url https://test.pypi.org/simple/ pyopengv
```

### Upload to PyPI (Production)

Once tested on TestPyPI:

```bash
# Using helper script
./build_wheel.sh upload

# Or manually
python -m build
twine upload dist/*
```

### Automated Publishing with GitHub Actions (Optional)

For automated releases, you can set up GitHub Actions to publish on tag push.

Example workflow:
```yaml
name: Publish Python Package

on:
  release:
    types: [published]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.9'
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install build twine
    - name: Build package
      run: python -m build
    - name: Publish to PyPI
      env:
        TWINE_USERNAME: __token__
        TWINE_PASSWORD: ${{ secrets.PYPI_API_TOKEN }}
      run: twine upload dist/*
```

## Project Structure

```
opengv/
├── pyproject.toml          # Modern Python package configuration
├── setup.py                # Build script with CMake integration
├── MANIFEST.in             # Files to include in distribution
├── build_wheel.sh          # Helper script for building/publishing
├── .pypirc.template        # Template for PyPI credentials
├── README_PYTHON.md        # Python package README
├── CMakeLists.txt          # CMake build configuration
├── src/                    # C++ source code
├── include/                # C++ header files
└── python/                 # Python bindings
    ├── pyopengv.cpp        # pybind11 bindings
    ├── CMakeLists.txt      # Python-specific CMake config
    └── tests.py            # Python tests
```

## Troubleshooting

### CMake not found

Install CMake:
```bash
pip install cmake
# or
sudo apt-get install cmake  # Ubuntu/Debian
brew install cmake          # macOS
```

### Eigen library not found

Install Eigen3:
```bash
sudo apt-get install libeigen3-dev  # Ubuntu/Debian
brew install eigen                  # macOS
```

### pybind11 not found

```bash
pip install pybind11
```

### Build fails with compiler errors

Ensure you have a C++11 compatible compiler:
```bash
# Ubuntu/Debian
sudo apt-get install build-essential

# macOS (install Xcode command line tools)
xcode-select --install
```

### Wheel platform compatibility

The wheel is built for your specific platform. For multi-platform support:

1. Use cibuildwheel for automated multi-platform builds
2. Build on each target platform separately
3. Consider manylinux containers for Linux compatibility

### Import errors after installation

Check that the wheel installed correctly:
```bash
pip show pyopengv
python -c "import pyopengv; print(pyopengv.__file__)"
```

## Version Management

Version is specified in `pyproject.toml`. To release a new version:

1. Update version in `pyproject.toml`:
   ```toml
   [project]
   version = "1.1.0"
   ```

2. Build and test:
   ```bash
   ./build_wheel.sh build
   ./build_wheel.sh test
   ```

3. Upload to TestPyPI first:
   ```bash
   ./build_wheel.sh upload-test
   ```

4. If tests pass, upload to PyPI:
   ```bash
   ./build_wheel.sh upload
   ```

## Additional Resources

- PyPI Publishing Guide: https://packaging.python.org/tutorials/packaging-projects/
- Building C++ Extensions: https://setuptools.pypa.io/en/latest/userguide/ext_modules.html
- pybind11 Documentation: https://pybind11.readthedocs.io/
- CMake Documentation: https://cmake.org/documentation/
