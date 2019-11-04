cmake_minimum_required(VERSION 2.8.3)
project(example_package)

add_compile_options(-std=c++14)

#a dummy_target so qtcreator have access to all files
######################################################################### 
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

#########################################################################


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
#find_package(aruco REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   SetVelocities.msg
 )

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    SetTask.srv
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   visualization_msgs
   geometry_msgs
 )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs visualization_msgs geometry_msgs sensor_msgs message_runtime cv_bridge image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
 include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})  # ${aruco_INCLUDE_DIRS}

# Declare a C++ library
 add_library(example_package src/example_package.cpp)

# Add cmake target dependencies of the library
 add_dependencies(example_package ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 # Specify libraries to link a library target against
 target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${OpenCV_LIBS}) # ${aruco_LIBS}

# Declare a C++ executable
 add_executable(${PROJECT_NAME}_node nodes/example_package_node.cpp)

# Add cmake target dependencies of the executable
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link an executable target against
 target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
 install(TARGETS example_package example_package_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_example_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
