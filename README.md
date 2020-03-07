# Extensible framework of lidar mapping and localization

This is a framework of lidar mapping and localization with strong extensibility. Mapping and matching are separate parts of the framework. Mapping contains data pretreat, front end, back end, loop closing and viewer. Matching contains data pretreat and map matching. The characteristic of this framework is that most of it's modules can be replaced very easily with the help of polymorphism in c++. For example, you can replace ndt with icp in the front end, or replace g2o with gtsam in the back end without change code in the flow. Therefor, it is a good framework for research.

<img src="https://github.com/Little-Potato-1990/lidar_localization/blob/master/picture/kitti.jpg" width = 55% height = 55%/>

**Modifier:** [Qian Ren](https://www.zhihu.com/people/ren-gan-16)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04.
ROS Kinetic.
```
sudo apt-get install ros-kinetic-desktop-full
```


### 1.2. **G2O**
To avoid version problems, we provide the setup file of g2o in the setup_file directory. Install it follow the readme in [G2O Installation](https://github.com/RainerKuemmerle/g2o)

### 1.3. **PCL**
```
sudo apt-get install libpcl-all
```

### 1.4. **kitti2bag**
```
sudo pip install -U numpy
sudo pip install kitti2bag
```

## 2. Build lidar_localization
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/Little-Potato-1990/lidar_localization
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 4. KITTI Example (Velodyne HDL-64)
### 4.1 KITTI Data to Bagfile
Download [KITTI raw_data dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to DATASET_PATH.       
For example “2011_10_03_drive_0027_sync.zip” and “2011_10_03_calib.zip”(the calibration file is necessary), unzip the files and put them according to this      
<img src="https://github.com/Little-Potato-1990/lidar_localization/blob/master/picture/kitti2bag.jpg" width = 55% height = 55%/>        
then run
```
cd DATASET_PATH
kitti2bag -t 2011_10_03 -r 0027 raw_synced
```
then we will get kitti_2011_10_03_drive_0027_synced.bag in the directory.

### 4.2 Mapping
run
```
roslaunch lidar_localization mapping.launch
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```     
then mapping begin      
<img src="https://github.com/Little-Potato-1990/lidar_localization/blob/master/picture/mapping.gif" width = 720 height = 351/>      
when the saw mapping is over in rviz, run 
```
rosservice call /optimize_map
```
then the full cloud map will display in rviz.   
to save map to pcd file, run
```
rosservice call /save_map
```
then "map.pcd" and "filtered_map.pcd" will be saved in the directory "lidar_localization/slam_data/map"

### 4.3 Matching
input the path of "filtered_map.pcd" in "config/matching/matching.yaml" with the param "map_path",run
```
roslaunch lidar_localization matching.launch
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```
then the matching begin     
<img src="https://github.com/Little-Potato-1990/lidar_localization/blob/master/picture/matching.gif" width = 720 height = 351/>

## 5. Extend the Framework
### 5.1  Replace Module
the modules of the mapping part is separate, we can replace anyone of them. For example, the front end module can be replaced with [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM), when you have compile the project of A-LOAM, you can run 
```
roslaunch lidar_localization mapping_with_aloam.launch
```
in this project. Further more, you can replace it with any other lidar odom.

### 5.2 Add New Algorithms
the pointcloud registration is NDT in the frong end, if you want use ICP, you can add another child class of the virtual base class named "PointCloudInterface". Follow this method, you can add many other algorithms in the directory "lidar_localization/models".