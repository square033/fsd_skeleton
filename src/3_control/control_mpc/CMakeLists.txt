cmake_minimum_required(VERSION 2.8.3)
project(control_mpc)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  fsd_common_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  fsd_common_msgs
  geometry_msgs
  )

find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CASADI REQUIRED casadi)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/mpc_handle.cpp
  src/mpc.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARIES} 
  )
