
Cartographer ROS Integration
============================

Purpose
=======

[`Cartographer`](https://github.com/cartographer-project/cartographer) is a system that provides real-time simultaneous localization
and mapping ([`SLAM`](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)) in 2D and 3D across multiple platforms and sensor
configurations. This project provides Cartographer's ROS integration.

Getting started
===============

* Learn to use Cartographer with ROS at [`our Read the Docs site`](https://google-cartographer-ros.readthedocs.io).
* You can ask a question by [`creating an issue`](https://github.com/cartographer-project/cartographer_ros/issues/new?labels=question).

My note
===============
- 参考博客：
    - [cartographer 超详细注释代码](https://github.com/xiangli0608/cartographer_detailed_comments_ws)
    - [无处不在的小土：Cartographer源码解读](https://gaoyichao.com/Xiaotu/?book=Cartographer%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=index)
    - [知乎-AprilLee：cartographer源码详细解读](https://www.zhihu.com/column/c_1040559544505704448)
- 函数关系：[飞书文档：Cartographer函数](https://b2ggynj5jr.feishu.cn/mindnotes/bmncnT9KdRDtQvNqVeorjjhamff)


运行rosbag
=========
1. 2D demo
```
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```
2. Pure localization
    1. Generate the map
    ```
    roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
    ```
    2. pure localization
    ```
    roslaunch cartographer_ros demo_backpack_2d_localization.launch \
   load_state_filename:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag.pbstream \
   bag_filename:=${HOME}/Downloads/b2-2016-04-27-12-31-41.bag
    ```

2. Revo LDS: an example bag captured from a low-cost Revo Laser Distance Sensor from Neato Robotics vacuum cleaners
```
roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag
```
3. Taurob Tracker: an example bag captured from a Taurob Tracker teleoperation robot
```
roslaunch cartographer_ros demo_taurob_tracker.launch bag_filename:=${HOME}/Downloads/taurob_tracker_simulation.bag
```

Contributing
============

You can find information about contributing to Cartographer's ROS integration
at [`our Contribution page`](https://github.com/cartographer-project/cartographer_ros/blob/master/CONTRIBUTING.md).

