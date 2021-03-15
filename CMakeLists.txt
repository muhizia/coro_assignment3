cmake_minimum_required(VERSION 3.0.2)
project(assignment3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  roslib
  lynxmotion_al5d_description
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(${PROJECT_NAME}_amuhizi src/amuhiziImplementation.cpp src/amuhiziApplication.cpp)
set_target_properties(${PROJECT_NAME}_amuhizi PROPERTIES OUTPUT_NAME amuhizi  PREFIX "")

# Install data files
install(DIRECTORY data/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

target_link_libraries(${PROJECT_NAME}_amuhizi ${catkin_LIBRARIES})