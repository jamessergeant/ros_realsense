cmake_minimum_required(VERSION 2.8.3)
project(capsicum_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
  message_filters
  pcl_ros
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(OpenMP)
#find_package(PCL REQUIRED)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomPCLConfig.cmake")
  set(PCL_CONFIGURED true)
  message(STATUS "pcl: cmake/CustomPCLConfig.cmake found, project will be built.")
else()
  set(PCL_CONFIGURED false)
  message(WARNING "pcl: cmake/CustomPCLConfig.cmake not found, project won't be built.")
endif()

if (${PCL_CONFIGURED})
  include(cmake/CustomPCLConfig.cmake)
  message(STATUS "pcl: Looking for PCL configuration file at " ${PCL_DIR})
  find_package(PCL CONFIG REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

endif()

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################

## Generate added messages and services with any dependencies listed here
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   segmentCapsicum.srv
 )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  capsicum_detection
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES capsicum_segmentation capsicum_depth_filter realsense_registration
#  CATKIN_DEPENDS baxter_core_msgs cv_bridge moveit_core roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}

)

## Declare a cpp library
# add_library(fruit_picking_test
#   src/${PROJECT_NAME}/fruit_picking_test.cpp
# )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
 add_executable(capsicum_segmentation src/capsicum_segmentation.cpp src/capsicum_detector.cpp src/cloud_filtering.cpp src/cloud_segmentation.cpp)

 add_executable(capsicum_depth_filter src/capsicum_depth_filter.cpp src/capsicum_detector.cpp src/cloud_filtering.cpp src/cloud_segmentation.cpp)

 add_executable(realsense_registration src/realsense_registration.cpp)

 add_library(capsicum_filtering_tools src/capsicum_detector.cpp src/cloud_filtering_tools.cpp)

## Add cmake target dependenciesolour_space.h"
#include <c of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies( capsicum_segmentation ${PROJECT_NAME}_generate_messages_cpp )

## Specify libraries to link a library or executable target against
 target_link_libraries(capsicum_segmentation
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
 )
## Specify libraries to link a library or executable target against
 target_link_libraries(capsicum_depth_filter
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
 )

 target_link_libraries(realsense_registration
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
 )

 target_link_libraries(capsicum_filtering_tools
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS fruit_picking_test fruit_picking_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fruit_picking_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
