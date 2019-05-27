# README #

This package is for running the semantic SLAM algorithm using planar extracted planar surfaces and the received detections (under development). Currently the package can use the following objects to create a semantic map:

- chair
- tvmonitor
- book
- keyboard
- laptop
- bucket
- car



### How do I get set up? ###

To try a simple example with blue bucket detector create a ros workspace and clone the following packages:

- download the rosbag from the link - 
- mkdir -p workspace/ros/semantic_slam_ws/src/ && cd workspace/ros/semantic_slam_ws/src/
- git clone https://bitbucket.org/hridaybavle/semantic_slam.git && git clone https://bitbucket.org/hridaybavle/bucket_detector.git
- cd .. && catkin_make -DCMAKE_BUILD_TYPE=Relase
-  gedit src/sematic_slam/launchers/ps_slam_with_snap_pose_bucket_det_lab_data.launch 
- *insert the rosbag location in the launcher*
- source devel/setup.bash
- roslaunch semantic_slam ps_slam_with_snap_pose_bucket_det_lab_data.launch 
- rviz -d src/semantic_slam/rviz/graph_semantic_slam.rviz

### Subsribed Topics ### 

- **/SQ04/snap_vislam/vislam/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
The default snapdragon VIO pose published in NED in frame. This message can be remapped remapped to any other VO pose message publishing in NED frame. ([See frame conventions](https://en.wikipedia.org/wiki/Axes_conventions))


- **/rovio/odometry** ([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))  
The VIO odometry published in ENU frame. Can be remapped to the desired topic name in the launcher. 


- **/depth_registered/points** ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) 
The point cloud required for planar surface extraction. 


- **/darknet_ros/bounding_boxes**([darknet_msgs_ros/BoundingBoxes](https://github.com/leggedrobotics/darknet_ros))  
The detection bounding boxes published by yolo if using the yolo detector ros package. 

- **/image_processed/bounding_boxes**([ShapeColor_ObjectDetection/DetectedObjects](https://hridaybavle@bitbucket.org/hridaybavle/bucket_detector.git)
The detection bounding boxes if using the bucket detector.
