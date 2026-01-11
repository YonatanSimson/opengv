
# Configure Python before finding pybind11 to ensure correct Python is used
if(PYTHON_EXECUTABLE AND NOT Python3_FOUND)
  # Force pybind11 to use the specified Python
  set(Python3_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE PATH "Python executable")
  get_filename_component(_PYTHON_DIR "${PYTHON_EXECUTABLE}" DIRECTORY)
  get_filename_component(_PYTHON_PREFIX "${_PYTHON_DIR}" DIRECTORY)
  set(Python3_ROOT_DIR ${_PYTHON_PREFIX} CACHE PATH "Python root directory")
  # Clear any cached Python3 variables to force re-detection
  unset(Python3_FOUND CACHE)
  unset(Python3_LIBRARIES CACHE)
  unset(Python3_INCLUDE_DIRS CACHE)
endif()

# Find pybind11 - it can be installed via conda, vcpkg, or as a submodule
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  # Try to find pybind11 as a submodule
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pybind11/CMakeLists.txt")
    message(STATUS "Found pybind11 as submodule, adding subdirectory")
    add_subdirectory(pybind11)
    set(pybind11_FOUND TRUE)
  else()
    # Try to find headers manually for better error messages
    find_path(PYBIND11_INCLUDE_DIR pybind11/pybind11.h
      PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/pybind11
        ${CMAKE_CURRENT_SOURCE_DIR}/../pybind11
        ${CMAKE_INSTALL_PREFIX}/include
        $ENV{CONDA_PREFIX}/include
      PATH_SUFFIXES include
    )
    if(PYBIND11_INCLUDE_DIR)
      message(FATAL_ERROR "pybind11 headers found at ${PYBIND11_INCLUDE_DIR} but CMake package not available. Please install pybind11 properly via: conda install pybind11 OR pip install pybind11")
    else()
      message(FATAL_ERROR "pybind11 not found. Please install it via: conda install pybind11, pip install pybind11, vcpkg install pybind11, or add it as a git submodule.")
    endif()
  endif()
endif()

# Include Python include directories if available
if(PYTHON_INCLUDE_DIRS)
  include_directories(${PYTHON_INCLUDE_DIRS})
elseif(Python_INCLUDE_DIRS)
  include_directories(${Python_INCLUDE_DIRS})
endif()

# Create the Python module using pybind11
# pybind11_add_module should be available if find_package succeeded
pybind11_add_module(pyopengv pyopengv.cpp)

# Link libraries - use manual linking on Windows to avoid pybind11's Python detection issues
if(WIN32 AND PYTHON_EXECUTABLE)
  get_filename_component(_PYTHON_DIR "${PYTHON_EXECUTABLE}" DIRECTORY)
  get_filename_component(_PYTHON_PREFIX "${_PYTHON_DIR}" DIRECTORY)
  
  # Get Python version to find correct library
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')"
    OUTPUT_VARIABLE _PYTHON_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  # Find the correct Python library
  find_library(CORRECT_PYTHON_LIB
    NAMES python${_PYTHON_VERSION} python3 python313 python312 python311 python310 python39
    PATHS
      ${_PYTHON_PREFIX}/libs
      ${_PYTHON_DIR}/libs
    NO_DEFAULT_PATH
  )
  
  if(CORRECT_PYTHON_LIB)
    # Disable LTO for lld-link compatibility (use compile option, not link option)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(pyopengv PRIVATE -fno-lto)
    endif()

    # Link opengv and Python library
    # Note: pybind11_add_module already sets up basic linking, we just add opengv
    target_link_libraries(pyopengv PRIVATE opengv ${CORRECT_PYTHON_LIB})
    message(STATUS "Using Python library: ${CORRECT_PYTHON_LIB}")
  else()
    # Fallback to standard linking
    target_link_libraries(pyopengv PRIVATE opengv pybind11::module)
  endif()
else()
  # Non-Windows: use standard pybind11 linking
  target_link_libraries(pyopengv PRIVATE opengv pybind11::module)
endif()

# Set output properties for Windows (use .pyd extension)
if(WIN32)
  set_target_properties(pyopengv PROPERTIES SUFFIX ".pyd")
  # Ensure we build a shared library
  set_target_properties(pyopengv PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )
endif()

# Find where to install python libs - use modern sysconfig instead of deprecated distutils
execute_process(COMMAND
    ${PYTHON_EXECUTABLE} -c 
    "import sys; import sysconfig; import os; print(os.path.relpath(sysconfig.get_path('purelib', sysconfig.get_default_scheme()), sys.prefix).replace(os.sep, '/'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES 
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# Fallback for older Python versions that don't have get_default_scheme
if(NOT PYTHON_SITE_PACKAGES OR PYTHON_SITE_PACKAGES STREQUAL "")
  execute_process(COMMAND
      ${PYTHON_EXECUTABLE} -c 
      "import sys; import sysconfig; import os; libdir = sysconfig.get_path('purelib'); print(os.path.relpath(libdir, sys.prefix).replace(os.sep, '/'))"
      OUTPUT_VARIABLE PYTHON_SITE_PACKAGES 
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
  )
endif()

# Final fallback - use a sensible default
if(NOT PYTHON_SITE_PACKAGES OR PYTHON_SITE_PACKAGES STREQUAL "")
  if(WIN32)
    set(PYTHON_SITE_PACKAGES "Lib/site-packages")
  else()
    set(PYTHON_SITE_PACKAGES "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
  endif()
  message(WARNING "Could not determine Python site-packages directory, using default: ${PYTHON_SITE_PACKAGES}")
endif()

set(PYTHON_INSTALL_DIR
    "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}"
    CACHE PATH "Path where to install pyopengv")

install(TARGETS pyopengv 
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}"
    RUNTIME DESTINATION "${PYTHON_INSTALL_DIR}"  # For Windows .pyd files
    ARCHIVE DESTINATION "${PYTHON_INSTALL_DIR}")

message(STATUS "Python executable: ${PYTHON_EXECUTABLE}")
message(STATUS "Python install directory: ${PYTHON_INSTALL_DIR}")