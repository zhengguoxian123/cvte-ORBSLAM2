# modified by @rongbohou
Based on the version of MathewDenny http://github.com/MathewDenny
Add the 2d map using scan data.

# usage 
1. localization : "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings if_reuse_map if_publish_tf" 

$: roslaunch kinect2_bridge kinect2_bridge.launch
$: rosrun orbslam2 localization /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml false false

2. mapping: build 2d and 3d map simultaneously
rosrun orbslam2 mapping /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml false false

2. evaluation:
rosrun orbslam2 evaluation /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml false false
