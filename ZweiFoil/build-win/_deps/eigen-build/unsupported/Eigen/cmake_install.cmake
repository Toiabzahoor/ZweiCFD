# Install script for directory: /workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/x86_64-w64-mingw32-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/AdolcForward"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/AlignedVector3"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/ArpackSupport"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/AutoDiff"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/BVH"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/EulerAngles"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/FFT"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/IterativeSolvers"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/KroneckerProduct"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/LevenbergMarquardt"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/MatrixFunctions"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/MoreVectorization"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/MPRealSupport"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/NonLinearOptimization"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/NumericalDiff"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/OpenGLSupport"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/Polynomials"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/Skyline"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/SparseExtra"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/SpecialFunctions"
    "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/Splines"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-src/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/eigen-build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

