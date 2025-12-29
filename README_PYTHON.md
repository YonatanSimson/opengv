# PyOpenGV - Python Bindings for OpenGV

Python bindings for OpenGV, a library for solving calibrated central and non-central geometric vision problems.

## Features

- **Absolute Pose**: P3P, EPnP, UPnP, GPnP, SQPnP solvers
- **Relative Pose**: 5-point, 7-point, 8-point, eigensolver methods
- **Omnidirectional Support**: Angular consistency validation for panoramic cameras
- **RANSAC Integration**: Robust estimation with sample consensus
- **Triangulation**: Multiple methods for 3D point reconstruction

## Installation

### From PyPI (when published)

```bash
pip install pyopengv
```

### From Source

1. Install dependencies:
```bash
./build_wheel.sh install-deps
```

2. Build and install:
```bash
./build_wheel.sh build
./build_wheel.sh install-wheel
```

Or for development:
```bash
pip install -e .
```

## Quick Start

```python
import numpy as np
import pyopengv

# Example: Absolute pose estimation with P3P
bearings = np.random.randn(3, 3)  # 3D bearing vectors
points = np.random.randn(3, 3)     # 3D world points

# Normalize bearings
bearings = bearings / np.linalg.norm(bearings, axis=1, keepdims=True)

# Compute pose
transformation = pyopengv.absolute_pose.p3p_kneip(bearings, points)
```

## Building Wheels

The repository includes a helper script for building and distributing wheels:

```bash
# Build wheel locally
./build_wheel.sh build

# Install from built wheel
./build_wheel.sh install-wheel

# Upload to TestPyPI (for testing)
./build_wheel.sh upload-test

# Upload to PyPI (production)
./build_wheel.sh upload
```

## Requirements

- Python >= 3.7
- NumPy >= 1.19.0
- CMake >= 3.15 (for building from source)
- C++11 compatible compiler
- Eigen3 library

## License

BSD 3-Clause License. See LICENSE.txt for details.

## Citation

If you use OpenGV in your research, please cite:

```
@article{kneip2014opengv,
  title={OpenGV: A unified and generalized approach to real-time calibrated geometric vision},
  author={Kneip, Laurent and Furgale, Paul},
  journal={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2014}
}
```

## Links

- Homepage: http://laurentkneip.github.io/opengv
- Documentation: http://laurentkneip.github.io/opengv
- Repository: https://github.com/laurentkneip/opengv
