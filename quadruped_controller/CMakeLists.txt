cmake_minimum_required(VERSION 3.16.3)
project(quadruped_controller)

# add compile options
add_compile_options(-Wall -Wextra)

# Compile as C++20
set(CMAKE_CXX_STANDARD 20)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadruped_msgs
  roscpp
  sensor_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Armadillo REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(qpOASES MODULE REQUIRED)

# message(STATUS "QP : " ${QPOASES_FOUND})
# message(STATUS "QP lib: " ${qpOASES_LIBRARIES})
# message(STATUS "QP include: " ${qpOASES_INCLUDE_DIRS})

# if (qpOASES_FOUND)
#   message(STATUS "Found qpOASES")
# else ()
#   message(ERROR "NO QP lib")
# endif ()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
  geometry_msgs
  quadruped_msgs
  roscpp
  sensor_msgs
  std_srvs
 DEPENDS
  ARMADILLO
  drake
  qpOASES
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/balance_controller.cpp
  src/${PROJECT_NAME}/joint_controller.cpp
  src/${PROJECT_NAME}/kinematics.cpp
  src/${PROJECT_NAME}/trajectory.cpp
  src/${PROJECT_NAME}/math/numerics.cpp
  src/${PROJECT_NAME}/math/rigid3d.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(commander src/commander_node.cpp)
# add_executable(test_qp src/test_qp_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(test_qp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${qpOASES_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  drake::drake
)

target_link_libraries(commander
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ARMADILLO_LIBRARIES}
)

# target_link_libraries(test_qp
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
#   ${ARMADILLO_LIBRARIES}
#   # ${qpOASES_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS commander
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_quadruped_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
