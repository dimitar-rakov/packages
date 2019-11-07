cmake_minimum_required(VERSION 2.8.3)
project(visual_features_extractor)

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

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
  tf
  tf_conversions
  control_toolbox
  nodelet
  pluginlib
)

# Eigen
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)


##############################################################################


################################################
## Declare ROS messages, services and actions ##
################################################


# Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   VisFeature.msg
)
# Generate services in the 'srv' folder
 add_service_files(
   DIRECTORY srv
   FILES
   SetTF.srv
   SetDepth.srv
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
        std_msgs
        geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS cv_bridge image_transport message_runtime roscpp tf tf_conversions std_msgs std_srvs geometry_msgs nodelet pluginlib
)

###########
## Build ##
###########

## Specify additional locations of header files
 include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${aruco_INCLUDE_DIRS})

# Declare a C++ library
 add_library(${PROJECT_NAME} src/visual_features_extractor.cpp  src/visual_features_extractor_nodelet.cpp )

# Add cmake target dependencies of the executable
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})

# Declare a C++ executable
 add_executable(${PROJECT_NAME}_node nodes/visual_features_extractor_node.cpp)

# Add cmake target dependencies of the executable, same as for the library above
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${aruco_LIBS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS visual_features_extractor visual_features_extractor_node
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

## Mark other files for installation (e.g. launch and bag files, etc.)
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_visual_features_extractor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
