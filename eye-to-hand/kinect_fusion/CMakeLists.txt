cmake_minimum_required(VERSION 2.8.3)
project(kinect_fusion)

add_compile_options(-std=c++14)
set(CMAKE_MODULE_PATH /usr/local/lib/cmake)

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
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  nodelet
  pluginlib
)


find_package(OpenCV REQUIRED)
#find_package(aruco REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)


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
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime cv_bridge image_transport pcl_conversions pcl_ros nodelet pluginlib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Declare a C++ library
 add_library(${PROJECT_NAME}
   src/kinect_fusion.cpp
   src/kinect_fusion_nodelet.cpp
 )

# Add cmake target dependencies of the library
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Declare a C++ executable target against
 add_executable(${PROJECT_NAME}_node nodes/kinect_fusion_node.cpp)

# Add cmake target dependencies of the executable
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link an executable target against
 target_link_libraries(${PROJECT_NAME}_node  ${PROJECT_NAME}
 )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS kinect_fusion kinect_fusion_node
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

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinect_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
