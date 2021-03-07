# Quadruped Control 
Based on ETH Zuirch robots HyQ and StarlETH and MIT cheetah. 


# Development Environment 
- Ubunut 20.04 
- ROS Noetic 
- GCC 9.3.0

# Dependencies
- [Armadillo](http://arma.sourceforge.net/)
- [drake](https://github.com/RobotLocomotion/drake)
- [qpOASES](https://github.com/coin-or/qpOASES)

## Optional Dependencies 
For improved performance in both Armadillo and qpOASES install first [OpenBlas](https://github.com/xianyi/OpenBLAS) and [LAPACK](https://github.com/Reference-LAPACK/lapack). See Armadillo's install [notes](http://arma.sourceforge.net/download.html).

**Important**: The current version of qpOASES only creates a static library and will 
cause Armadillo to segfault. The sagfault might be because qpOASES redefines internally some BLAS/LAPACK functions, and this can create problems if linking with another library that uses BLAS or LAPACK. For more info see [PR #108](https://github.com/coin-or/qpOASES/pull/108) and [PR #109](https://github.com/coin-or/qpOASES/pull/109).

To properly build qpOASES add/modify the following lines to the CMakeLists.txt.

```cmake
# Avoids names conflicting with BLAS/LAPACK
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__AVOID_LA_NAMING_CONFLICTS__")

# Remove STATIC to build a shared library
ADD_LIBRARY(qpOASES ${SRC})
```

## Dependency Versions Used 
- OpenBlas 0.3.13 
- LAPACK 3.9.0
- Armadillo 10.2.1
- drake v0.26.0
- qpOASES master (SHAID: 326a651)

