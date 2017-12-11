cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME semantic_SLAM)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). NO! Not working with PCL
add_definitions(-std=c++11)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo




set(SEMANTIC_SLAM_SOURCE_DIR
    src/sources)

set(SEMANTIC_SLAM_INCLUDE_DIR
    src/include
    )

set(SEMANTIC_SLAM_SOURCE_FILES
    #General
    src/source/semantic_SLAM.cpp
		src/source/semantic_SLAM_node.cpp
    )


set(SEMANTIC_SLAM_SLAM_HEADER_FILES
    #General
    src/include/semantic_SLAM.h
    )

find_package(catkin REQUIRED
    COMPONENTS roscpp)

find_package(OpenCV REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
  message(WARN "Using Eigen2!")
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
    INCLUDE_DIRS ${SEMANTIC_SLAM_INCLUDE_DIR}
    LIBRARIES semantic_SLAM OpenCV 
    CATKIN_DEPENDS roscpp
    )


include_directories(${SEMANTIC_SLAM_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_library(SEMANTIC_SLAM ${SEMANTIC_SLAM_SOURCE_FILES} ${SEMANTIC_SLAM_HEADER_FILES})
add_dependencies(SEMANTIC_SLAM ${catkin_EXPORTED_TARGETS})
target_link_libraries(SEMANTIC_SLAM ${catkin_LIBRARIES})
target_link_libraries(SEMANTIC_SLAM ${EIGEN3_LIBRARIES})


add_executable(${PROJECT_NAME}_node ${SEMANTIC_SLAM_SOURCE_FILES} ${SEMANTIC_SLAM_HEADER_FILES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node SEMANTIC_SLAM)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


