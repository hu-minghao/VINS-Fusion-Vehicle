# VINS-Fusion for For Vehicle dense mapping and navigation

This is a specific verision of VINS-Fusion( based on https://github.com/HKUST-Aerial-Robotics/VINS-Fusion ) that supports RGB-D sensors and Wheel Encoder. Some codes we reference the project VINS-Fusion-RGBD (https://github.com/ManiiXu/VINS-Fusion-RGBD.git)

## 一种将轮式编码器数据接入前端，并利用RGB-D传感器进行建图的VINS-Fusion算法

1、只利用轮式编码器和RGB-D传感器数据，禁用了imu，不会飞飘  
在使用realsense-d435i作为输入传感器并使用了内置imu时，VINS-Fusion轨迹很容易飞飘，进行分析时发现视觉惯性紧耦合优化时求解结果
都没有收敛，如果增加迭代次数和求解时间，优化结果会收敛好一些，但静止时轨迹任然容易飞走。加上小车上轮式编码器相比于imu和视觉里
程计，能提供一个更为精确的位移估计值，所以这里直接禁用了imu数据，并将轮式编码器的数据代替视觉里程计数据作为前端估计值，定位和建图
鲁棒性更好。

2、octomap点云增量建图并实时生成导航的栅格地图  
本算法只发布当前帧的RGB-D点云数据（点云数据经过一定的过滤），点云地图的拼接交给ROS的八叉树地图库octomap_service完成，这样
大规模建图中本算法的实时性更好，能以传感器为中心扫出正常的栅格地图。
octomap_service 通过catkin_make单独编译，并与本算法一起启动，注意修改octomap_service 启动文件中的话题，拼接后的点云和实时生成
的栅格地图通过octomap_service 的命令来保存。
  
3、任意地点加载地图后的重定位  
这里默认world与map是两个重合的坐标系，odom坐标系与world坐标系间的转换由本算法估计得出。在任意地点加载地图后，在没有与地图姿态
图回环时，world与odom是重合的，一旦发生回环，则本算法会计算odom与world间的位置关系，将小车定位到map坐标系下正确的坐标。

4、yaml文件中更多参数的配置
yaml中添加了轮式编码器话题，轮式编码器坐标与相机坐标的外参，回环时最小匹配特征点数，定位时是否只与加载的姿态图关键帧回环（agv_model
 location： 只与加载的地图姿态图关键帧回环，mapping:和所有的关键帧都可以回环），相关参数参考realsense_depth_imu_config_d435i_location.yaml。


5、只对小车上的RGB-D传感器进行了测试，效果还行，其他配置还没有进行过测试。

## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **OpenCV** 
OpenCV 2.4.8

### 1.3. **Ceres Solver** 
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.4. **PCL** 
PCL 1.7 

## 2. mapping
octomap_service [octomap github](https://github.com/OctoMap/octomap_mapping.git)


## 2. Build 
```
    cd ~/catkin_ws/src
    git clone https://github.com/ManiiXu/VINS-Fusion.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Example 

### 3.1 建图
roslaunch vins vins_d435i_map.launch
rosrun loop_fusion loop_fusion_node  /home/catkin_ws/src/VINS-Fusion/config/realsense/realsense_depth_imu_config_d435i.yaml
roslaunch octomap_service your_octomap.launch
roslaunch vins vins_rviz.launch

### 3.2 定位  
roslaunch loop_fusion loop_fusion_location.launch
rosrun vins vins_node  /home/catkin_ws/src/VINS-Fusion/config/realsense/realsense_depth_imu_config_d435i_location.yaml
roslaunch vins vins_rviz.launch



仅供学习。