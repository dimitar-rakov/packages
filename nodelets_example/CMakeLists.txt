cmake_minimum_required(VERSION 2.8.3)
project(nodelets_example)

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
  nodelet
  pluginlib
  visualization_msgs
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet pluginlib visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include  ${catkin_INCLUDE_DIRS})

# Declare a C++ library
 add_library(nodelets_example
     src/nodelets_example.cpp
     src/nodelets_example_nodelet.cpp
 )

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}) # ${OpenCV_LIBS} ${aruco_LIBS}

# Declare a C++ executable
 add_executable(${PROJECT_NAME}_node nodes/nodelets_example_node.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
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
 install(TARGETS nodelets_example nodelets_example_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nodelets_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
