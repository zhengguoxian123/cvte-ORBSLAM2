ORB-SLAM2
-------
## modified by @rongbohou
Based on the version of MathewDenny http://github.com/MathewDenny.  
Add the 2d mapping using scan data.

## 相关依赖库的安装

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11**.

安装 opencv2.412， 参考 http://www.samontab.com/web/2014/06/installing-opencv-2-4-9-in-ubuntu-14-04-lts/

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

```
git clone https://github.com/stevenlovegrove/Pangolin.git   
cd Pangolin  
mkdir build  
cd build  
cmake ..  
make -j
```


### ROS
We provide some examples to process the live input of a monocular or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.  

安装ros indigo，参考ROS官网 http://wiki.ros.org/indigo/Installation/Ubuntu

### g2o
https://github.com/RainerKuemmerle/g2o  
参考：http://www.cnblogs.com/gaoxiang12/p/4739934.html

### Sophus - Lie groups
用于直接法RGBD-DSLAM  

```
git clone https://github.com/strasdat/Sophus.git  
cd Sophus  
git checkout a621ff  
mkdir build  
cd build  
cmake ..  
make  
```

You don't need to install the library since cmake .. writes the package location to ~/.cmake/packages/ where CMake can later find it.

### Fast - Corner Detector
用于直接法RGBD-DSLAM  

```
git clone https://github.com/uzh-rpg/fast.git  
cd fast  
mkdir build  
cd build  
cmake ..  
make  
```

### vikit_common
用于直接法RGBD-DSLAM  
vikit contains camera models, some math and interpolation functions that SVO needs.
```
cd workspace  
git clone https://github.com/uzh-rpg/rpg_vikit.git  
```
in rpg_vikit/vikit_common/CMakeLists.txt set the flag USE_ROS to FALSE.  

```
cd rpg_vikit/vikit_common  
mkdir build  
cd build  
cmake ..  
make  
```

Sophus,Fast 和 vikit_common的安装参考：https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-Plain-CMake-(No-ROS)

##  Building library and some nodes
### 1. Creating a catkin Workspace

```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws/src  
catkin_init_workspace  
git clone https://github.com/rongbohou/cvte-ORBSLAM2.git
```

### 2. Building Thirdparty library
#### DBoW2
```
cd Thirdparty/DBoW2  
mkdir build  
cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release  
make -j  
```
#### g2o
```
cd Thirdparty/g2o  
mkdir build  
cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release  
make -j  
```
#### RGBD-DSLAM
```
cd Thirdparty/RGBD-DSLAM  
mkdir build  
cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release  
make -j  
```
### 3. Building package
```
cd ~/catkin_ws  
catkin_make --pkg orbslam2  
```
##  Usage
### 1. localization :   
```
"Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings if_reuse_map if_publish_tf if_use_odom"  
```
功能：在ORB-SLAM定位模式下跟踪机器人并发布tf用于导航。当ORB-SLAM跟丢时，利用机器人底座历程数据odom 或者 用直接法估计里程 进而发布tf继续用于导航。
如果超过一定时间的跟丢，机器人则执行原地旋转。

```
roslaunch kinect2_bridge kinect2_bridge.launch  
cd ~/rongbo/catkin_ws/orbslam2
rosrun orbslam2 localization /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml true true true
```

### 2. mapping: build 2d and 3d map simultaneously
功能：同时构建2d栅格概率地图和3d稀疏特征点地图，两地图是绝对对齐的。
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation mapping_orbslam_rplidar_2.launch

cd ~/rongbo/catkin_ws/orbslam2
rosrun orbslam2 mapping /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml false false
```
### 3. evaluation:
功能：利用键盘的非阻塞输入，保存测量点数据（位姿和特征点数量）到本地文件  
```
cd ~/rongbo/catkin_ws/orbslam2
rosrun orbslam2 evaluation /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml false false
```
