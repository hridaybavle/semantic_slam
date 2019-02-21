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
    src/source/plane_segmentation.cpp
    )


set(SEMANTIC_SLAM_HEADER_FILES
    #General
    src/include/semantic_SLAM.h
    src/include/plane_segmentation.h
    src/inlcude/tools.h
    )

find_package(catkin REQUIRED
    COMPONENTS roscpp std_msgs sensor_msgs image_transport tf_conversions tf particle_filter aruco_eye_msgs message_generation cv_bridge image_transport darknet_ros_msgs
    pcl_ros pcl_msgs pcl_conversions)

find_package(PCL 1.7 REQUIRED
    COMPONENTS common io sample_consensus segmentation filters)


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

add_message_files(
    FILES
    bebop/Ardrone3PilotingStateAttitudeChanged.msg
    ObjectInfo.msg
    DetectedObjects.msg
    MappedPointCloud.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS ${SEMANTIC_SLAM_INCLUDE_DIR}
    LIBRARIES semantic_SLAM OpenCV PCL
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport tf_conversions tf particle_filter aruco_eye_msgs message_runtime cv_bridge image_transport darknet_ros_msgs
    pcl_ros pcl_msgs pcl_conversions
    )


include_directories(${SEMANTIC_SLAM_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(SEMANTIC_SLAM ${SEMANTIC_SLAM_SOURCE_FILES})
add_dependencies(SEMANTIC_SLAM ${catkin_EXPORTED_TARGETS})
target_link_libraries(SEMANTIC_SLAM ${catkin_LIBRARIES})
target_link_libraries(SEMANTIC_SLAM ${EIGEN3_LIBRARIES})
target_link_libraries(SEMANTIC_SLAM ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES})

add_executable(${PROJECT_NAME}_node ${SEMANTIC_SLAM_SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node SEMANTIC_SLAM)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES})

add_executable(depth_synchronization_node src/source/depth_image_synchronizer.cpp)
add_dependencies(depth_synchronization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_synchronization_node ${OpenCV_LIBS})
target_link_libraries(depth_synchronization_node ${catkin_LIBRARIES})
