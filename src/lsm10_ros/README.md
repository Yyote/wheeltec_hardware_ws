# lsm10_ros

## version track
Author: yao


## Description
The `lsm10_ros` package is a linux ROS driver for lsm10.
The package is tested on Ubuntu 16.04 with ROS kinetic.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
source devel/setup.bash
```

**Published Topics**

``/scan`` (`sensor_msgs/scan`)
``/difop_information`` 

**Node**

```
roslaunch lsm10_v2 lsm10_v2.launch
```

rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 1"	 			(open lidar)
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 0"  			(close lidar)

rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 2"				(Radar does not filter)
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 3"				(Radar normal filtering)
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 4"				(Radar close range filtering)

## FAQ

## Bug Report







