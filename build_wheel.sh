#!/bin/bash
# Helper script for building and distributing pyopengv

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_help() {
    echo "PyOpenGV Build and Distribution Helper"
    echo ""
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  install-deps     Install build dependencies"
    echo "  build           Build wheel package locally"
    echo "  install         Install package locally (editable mode)"
    echo "  install-wheel   Install from built wheel"
    echo "  clean           Clean build artifacts"
    echo "  test            Run Python tests"
    echo "  upload-test     Upload to TestPyPI (for testing)"
    echo "  upload          Upload to PyPI (production)"
    echo "  help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 install-deps  # First time setup"
    echo "  $0 build         # Build wheel"
    echo "  $0 install       # Install for development"
    echo "  $0 upload-test   # Test upload to TestPyPI"
    echo "  $0 upload        # Upload to PyPI"
}

install_deps() {
    echo -e "${GREEN}Installing build dependencies...${NC}"
    pip install --upgrade pip setuptools wheel
    pip install --upgrade build twine
    pip install pybind11 numpy cmake ninja
    echo -e "${GREEN}Dependencies installed successfully!${NC}"
}

build_wheel() {
    echo -e "${GREEN}Building wheel package...${NC}"
    
    # Clean previous builds
    rm -rf dist/ build/ *.egg-info
    
    # Build wheel
    python -m build
    
    echo -e "${GREEN}Wheel built successfully!${NC}"
    echo -e "${YELLOW}Wheel files:${NC}"
    ls -lh dist/
}

install_editable() {
    echo -e "${GREEN}Installing package in editable mode...${NC}"
    pip install -e .
    echo -e "${GREEN}Package installed successfully!${NC}"
}

install_from_wheel() {
    echo -e "${GREEN}Installing from wheel...${NC}"
    
    if [ ! -d "dist" ] || [ -z "$(ls -A dist/*.whl 2>/dev/null)" ]; then
        echo -e "${RED}No wheel found. Building first...${NC}"
        build_wheel
    fi
    
    pip install dist/*.whl --force-reinstall
    echo -e "${GREEN}Package installed successfully!${NC}"
}

clean_build() {
    echo -e "${YELLOW}Cleaning build artifacts...${NC}"
    rm -rf build/
    rm -rf dist/
    rm -rf *.egg-info
    rm -rf python/*.egg-info
    find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
    find . -type f -name "*.pyc" -delete 2>/dev/null || true
    find . -type f -name "*.pyo" -delete 2>/dev/null || true
    echo -e "${GREEN}Clean completed!${NC}"
}

run_tests() {
    echo -e "${GREEN}Running Python tests...${NC}"
    cd python
    python tests.py
    cd ..
    echo -e "${GREEN}Tests completed!${NC}"
}

upload_testpypi() {
    echo -e "${YELLOW}Uploading to TestPyPI...${NC}"
    
    if [ ! -d "dist" ] || [ -z "$(ls -A dist/ 2>/dev/null)" ]; then
        echo -e "${RED}No distribution files found. Building first...${NC}"
        build_wheel
    fi
    
    echo -e "${YELLOW}Note: You'll need TestPyPI credentials${NC}"
    echo -e "${YELLOW}Register at: https://test.pypi.org/account/register/${NC}"
    
    twine upload --repository testpypi dist/*
    
    echo -e "${GREEN}Upload to TestPyPI completed!${NC}"
    echo -e "${YELLOW}Install with: pip install --index-url https://test.pypi.org/simple/ pyopengv${NC}"
}

upload_pypi() {
    echo -e "${RED}Uploading to PyPI (PRODUCTION)...${NC}"
    
    if [ ! -d "dist" ] || [ -z "$(ls -A dist/ 2>/dev/null)" ]; then
        echo -e "${RED}No distribution files found. Building first...${NC}"
        build_wheel
    fi
    
    echo -e "${YELLOW}⚠️  WARNING: This will upload to the PRODUCTION PyPI!${NC}"
    echo -e "${YELLOW}Make sure you've tested with TestPyPI first.${NC}"
    read -p "Are you sure you want to continue? (yes/no): " confirm
    
    if [ "$confirm" != "yes" ]; then
        echo -e "${RED}Upload cancelled.${NC}"
        exit 1
    fi
    
    echo -e "${YELLOW}Note: You'll need PyPI credentials${NC}"
    echo -e "${YELLOW}Register at: https://pypi.org/account/register/${NC}"
    
    twine upload dist/*
    
    echo -e "${GREEN}Upload to PyPI completed!${NC}"
    echo -e "${YELLOW}Install with: pip install pyopengv${NC}"
}

# Main command dispatcher
case "${1:-help}" in
    install-deps)
        install_deps
        ;;
    build)
        build_wheel
        ;;
    install)
        install_editable
        ;;
    install-wheel)
        install_from_wheel
        ;;
    clean)
        clean_build
        ;;
    test)
        run_tests
        ;;
    upload-test)
        upload_testpypi
        ;;
    upload)
        upload_pypi
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        echo ""
        print_help
        exit 1
        ;;
esac
