cmake_minimum_required(VERSION 3.0.2)
project(velocity_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  message_generation
  roscpp
  std_msgs
  gazebo_ros
  geometry_msgs
)

add_message_files(FILES wheel.msg)
	
# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs gazebo_msgs sensor_msgs roscpp)

# Declare a catkin package
catkin_package(LIBRARIES 

      ${PROJECT_NAME}
    CATKIN_DEPENDS 
      geometry_msgs
      std_msgs
      roscpp)


#Declare a C++ library

 #add_library(${publish_tutorial} src/${publish_tutorial}/publish.cpp)

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure

include_directories(include 
${catkin_INCLUDE_DIRS}
)



# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_library(${PROJECT_NAME} velocity_controller src/velocity.cpp)
add_executable(velocity_controller
  src/vel_con.cpp
)
add_dependencies(velocity_controller src/${PROJECT_NAME}_EXPORTED_TARGETS) 
target_link_libraries(velocity_controller ${catkin_LIBRARIES})




 

# Specify libraries to link a library or executable target against


