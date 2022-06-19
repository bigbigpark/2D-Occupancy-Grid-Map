cmake_minimum_required(VERSION 3.0.2)
project(laser_grid_mapping)

add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES a_ct_icp
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_grid_mapping src/laser_grid_mapping.cpp)
target_link_libraries(laser_grid_mapping
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
