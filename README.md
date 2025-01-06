# Wheeled-Robot-Enhancement-and-Practice

> Before you can reproduce these experiments, you must master the following [visualization tool](https://www.guyuehome.com/34172)

This is the coursework for the Zhejiang University Zhu Kezhen College Engineering High School course **Wheeled Robot Enhancement and Practice**, which is mainly divided into the following Labs.

![](assets/navi.gif)

## Running Guide

1. Create a new workspace (in the home directory):

> cd ~
>
> mkdir -p icp_ws/src

2. Put the folder course_agv_icp provided to everyone in the ~/icp_ws/src directory

3. Compile the workspace

> cd ~/icp_ws
>
> catkin_make
>
> source devel/setup.bash

4. Start the corresponding .launch file

> roslaunch course_agv_icp icp.launch

5. Run rosbag

> rosbag play xxx.bag

### Lab 1

Use `socket` to implement simple file sending and receiving

### Lab 2

Running and controlling turtles in the ROS system

### Lab 3  Motion Planning



==Report2搞过来==

### Lab 4 ICP Algorithm

.==word ICP搞过来==
