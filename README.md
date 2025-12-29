# OpenGV

OpenGV is a collection of computer vision methods for solving geometric vision problems. It contains absolute-pose, relative-pose, triangulation, and point-cloud alignment methods for the calibrated case.

**Author:** Laurent Kneip, ShanghaiTech, Mobile Perception Lab  
**Homepage:** http://laurentkneip.github.io/opengv

## Features

- **Absolute Pose Solvers**: P3P (Kneip, Gao), EPnP, UPnP, GPnP, **SQPnP**
- **Relative Pose Solvers**: 2-point, 5-point (Nister, Kneip), 7-point, 8-point, Eigensolver
- **RANSAC/LMedS Integration**: Robust estimation with outlier rejection
- **Omnidirectional Camera Support**: Full 360° bearing vector handling
- **Triangulation**: Multiple methods for 3D point reconstruction
- **Python Bindings**: Full API access via `pyopengv`

## Installation on macOS

### Prerequisites

Install the required dependencies using Homebrew:

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake eigen
```

### Building the C++ Library

```bash
# Clone the repository
git clone https://github.com/laurentkneip/opengv.git
cd opengv

# Initialize submodules (for Python bindings)
git submodule update --init --recursive

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build
make -j$(sysctl -n hw.ncpu)

# Run tests
make test
```

### Building with Python Bindings

```bash
# Configure with Python support
cmake .. -DBUILD_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)

# Build
make -j$(sysctl -n hw.ncpu)

# The Python module will be at: build/lib/pyopengv.cpython-*.so
```

### Installing the Python Wheel

The easiest way to install the Python bindings:

```bash
# Create a virtual environment (recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install numpy (required)
pip install numpy

# Build and install the wheel
pip install .
```

Or build a distributable wheel:

```bash
pip install wheel build
pip wheel . --no-deps -w dist/

# Install the wheel
pip install dist/pyopengv-*.whl
```

### Verify Installation

```bash
# Test C++ library
cd build
./bin/test_absolute_pose

# Test Python bindings
python3 -c "import pyopengv; print('pyopengv installed successfully!')"

# Run Python tests
PYTHONPATH=build/lib python3 python/tests.py
```

## Quick Start (Python)

```python
import numpy as np
import pyopengv

# Generate sample data
def normalized(x):
    return x / np.linalg.norm(x)

# Random camera pose
position = np.random.uniform(-2, 2, 3)
rotation = np.eye(3)  # Identity rotation

# Generate 3D points and their bearing vectors
num_points = 20
points = np.random.uniform(-5, 5, (num_points, 3))
bearings = np.array([normalized(rotation.T @ (p - position)) for p in points])

# Solve absolute pose with different methods
result_epnp = pyopengv.absolute_pose_epnp(bearings, points)
result_sqpnp = pyopengv.absolute_pose_sqpnp(bearings, points)

print("EPnP result:\n", result_epnp)
print("SQPnP result:\n", result_sqpnp)

# RANSAC with outliers
result_ransac = pyopengv.absolute_pose_ransac(
    bearings, points, "KNEIP", threshold=0.01, iterations=1000)
print("RANSAC result:\n", result_ransac)
```

## Available Python Functions

### Absolute Pose
- `absolute_pose_p3p_kneip(bearings, points)` - P3P Kneip solver
- `absolute_pose_p3p_gao(bearings, points)` - P3P Gao solver
- `absolute_pose_epnp(bearings, points)` - EPnP solver
- `absolute_pose_sqpnp(bearings, points)` - SQPnP solver (omnidirectional support)
- `absolute_pose_ransac(bearings, points, algo, threshold, iterations)` - RANSAC
- `absolute_pose_lmeds(bearings, points, algo, threshold, iterations)` - LMedS

**RANSAC/LMedS algorithms**: `"KNEIP"`, `"GAO"`, `"EPNP"`, `"SQPNP"`

### Relative Pose
- `relative_pose_twopt(b1, b2, rotation)` - 2-point with known rotation
- `relative_pose_fivept_nister(b1, b2)` - 5-point Nister
- `relative_pose_fivept_kneip(b1, b2)` - 5-point Kneip
- `relative_pose_sevenpt(b1, b2)` - 7-point algorithm
- `relative_pose_eightpt(b1, b2)` - 8-point algorithm
- `relative_pose_ransac(b1, b2, algo, threshold, iterations)` - RANSAC

### Triangulation
- `triangulation_triangulate(b1, b2, position, rotation)` - Triangulate points
- `triangulation_triangulate2(b1, b2, position, rotation)` - Alternative method

## Troubleshooting (macOS)

### Eigen not found
```bash
# Install Eigen
brew install eigen

# If CMake still can't find it, specify the path:
cmake .. -DEIGEN_INCLUDE_DIR=/opt/homebrew/include/eigen3
```

### C++14 required error
The library requires C++14. This should be handled automatically, but if you see errors:
```bash
cmake .. -DCMAKE_CXX_STANDARD=14
```

### Python module not found
Ensure you're using the correct Python:
```bash
cmake .. -DBUILD_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)
```

## Citation

If you use OpenGV in your research, please cite:

```bibtex
@inproceedings{kneip2014opengv,
  title={OpenGV: A unified and generalized approach to real-time calibrated geometric vision},
  author={Kneip, Laurent and Furgale, Paul},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2014}
}
```

For the SQPnP solver:

```bibtex
@inproceedings{terzakis2020sqpnp,
  title={A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem},
  author={Terzakis, George and Lourakis, Manolis},
  booktitle={European Conference on Computer Vision (ECCV)},
  year={2020}
}
```

## License

See [License.txt](License.txt) for details.

