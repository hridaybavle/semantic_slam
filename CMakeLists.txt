cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME semantic_SLAM)
project(${PROJECT_NAME})

add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
#set(CMAKE_CXX_FLAGS "-std=c++14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

set(SEMANTIC_SLAM_SOURCE_DIR
    src/sources)

set(SEMANTIC_SLAM_INCLUDE_DIR
    src/include
    )

FILE(GLOB_RECURSE SEMANTIC_SLAM_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/source/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/source/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/source/particle_filter/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/source/ps_graph_slam/*.cpp)

FILE(GLOB_RECURSE SEMANTIC_SLAM_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/g2o/*.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/include/particle_filter/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ps_graph_slam/*.hpp)


find_package(catkin REQUIRED
    COMPONENTS roscpp std_msgs sensor_msgs image_transport tf_conversions tf message_generation cv_bridge image_transport pcl_ros pcl_msgs pcl_conversions)

find_package(PCL 1.7 REQUIRED
    COMPONENTS common io sample_consensus segmentation filters)

find_package(G2O REQUIRED)
include_directories(SYSTEM PUBLIC ${G2O_INCLUDE_DIR})
link_directories(${G2O_LIBRARY_DIRS})

FIND_PACKAGE(Cholmod)
include_directories(${CHOLMOD_INCLUDE_DIR})

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


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
    ObjectInfo.msg
    DetectedObjects.msg
    acl_msgs/ViconState.msg
    BoundingBox.msg
    BoundingBoxes.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS ${SEMANTIC_SLAM_INCLUDE_DIR}
    LIBRARIES semantic_SLAM OpenCV PCL
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport tf_conversions tf message_runtime cv_bridge image_transport pcl_ros pcl_msgs pcl_conversions
    )


include_directories(${SEMANTIC_SLAM_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

add_library(SEMANTIC_SLAM ${SEMANTIC_SLAM_SOURCE_FILES} ${SEMANTIC_SLAM_HEADER_FILES})
add_dependencies(SEMANTIC_SLAM ${PROJECT_NAME}_gencpp)
add_dependencies(SEMANTIC_SLAM ${catkin_EXPORTED_TARGETS})
target_link_libraries(SEMANTIC_SLAM ${catkin_LIBRARIES})
target_link_libraries(SEMANTIC_SLAM ${EIGEN3_LIBRARIES})
target_link_libraries(SEMANTIC_SLAM ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES})
target_link_libraries(SEMANTIC_SLAM ${G2O_LIBS})


add_executable(semantic_graph_SLAM_node src/test/semantic_graph_SLAM_node.cpp)
add_dependencies(semantic_graph_SLAM_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(semantic_graph_SLAM_node SEMANTIC_SLAM)
target_link_libraries(semantic_graph_SLAM_node ${OpenCV_LIBS})
target_link_libraries(semantic_graph_SLAM_node ${catkin_LIBRARIES})

add_executable(depth_synchronization_node src/test/depth_image_synchronizer.cpp)
add_dependencies(depth_synchronization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_synchronization_node SEMANTIC_SLAM)
target_link_libraries(depth_synchronization_node ${OpenCV_LIBS})
target_link_libraries(depth_synchronization_node ${catkin_LIBRARIES})
