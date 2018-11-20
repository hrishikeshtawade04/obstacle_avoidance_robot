[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<h1 align=center> ENPM808X Obstacle Avoidance Robot </h1>
</p>
<p align="center">
<img src="/readme_images/turtlebot.png" height = 200>
</p>
</p>
<p align="center">
Reference for image: <a href='https://www.google.com/imgres?imgurl=https%3A%2F%2Fmoveit.ros.org%2Fassets%2Fimages%2Flogo%2FROS_logo.png&imgrefurl=https%3A%2F%2Fmoveit.ros.org%2F&docid=CyLySsR7n4CjkM&tbnid=yjk2FyriYEe3iM%3A&vet=10ahUKEwjLorK96a7eAhUBjVkKHcARAu0QMwhnKA0wDQ..i&w=680&h=365&bih=672&biw=1301&q=ROS%20image&ved=0ahUKEwjLorK96a7eAhUBjVkKHcARAu0QMwhnKA0wDQ&iact=mrc&uact=8' >link</a>
</p>

## Project Overview
The project runs a turtlebot in gazebo simulator.It works on two following nodes:
* turtlebot_gazebo : We use the turtlebot_world.launch file from this package to create environment in gazebo.
* walker : This node then publishes instructions to the turtlebot so that it can avoid obstacles in the gazebo environment.

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* Ubuntu 16.04
* Turtlebot packages

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

To install turtlebot packages run following command.
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
## Build Instructions
```
source /opt/ros/kinetic/setup.bash
sudo rm -R catkin_ws (skip if there is not catkin_ws)
mkdir -p catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone -b Week11_HW https://github.com/hrishikeshtawade04/obstacle_avoidance_robot.git
cd ..
catkin_make
```
## Running Instructions
To run the obstacle avoidance robot run the following launch file. This launch file will open the gazebo environment for  turtlebot and will also open the walker node which will then start publishing instructions to the turtlebot to avoid obstacles. You can also play with the  turtlebot by placing obstacles in real-time and see it avoiding those obstacles. Here the enableBag argument lets you record all the topics running during this instance execpt all /camera topics. Currently we have set it to false.
```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
cd <path to catkin_ws>
source devel/setup.bash
roslaunch obstacle_avoidance_robot run_robot.launch enableBag:=false
```
You can also run the nodes separately by following commands. First we will run roscore in a separate terminal.
```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
source /opt/ros/kinetic/setup.bash
roscore
```
Then open a new terminal and run the following command to open a new turtlebot environment in gazebo.
```
source /opt/ros/kinetic/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Then open a new terminal and launch following command to run the walker node.
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun obstacle_avoidance_robot walker
```
## Running Bag file
The commands to record turtlebot simulation data in bag file has already been added in the run_robot launch file which can be enabled to record by setting enableBag argument to true as below. The below set of commands record the data of all topics except /camera/* for 30 seconds in the results folder in storedTurtlebotData.bag file. To stop the recording press Ctrl-C in the new pop-up terminal and the main terminal where the command was executed.

```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
cd <path to catkin_ws>
source devel/setup.bash
roslaunch roslaunch obstacle_avoidance_robot run_robot.launch enableBag:=true
```
To verify the .bag file run the following command.
```
cd <path to repository>/results
rosbag info storedTurtlebotData.bag
```
It should give similar output as following.
```
path:        storedTurtlebotData.bag
version:     2.0
duration:    29.9s
start:       Dec 31 1969 19:00:00.28 (0.28)
end:         Dec 31 1969 19:00:30.20 (30.20)
size:        11.7 MB
messages:    23033
compression: none [16/16 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            2992 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/navi                            302 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               2989 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              2991 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     2863 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     58 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     287 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     2873 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  115 msgs    : bond/Status                           (3 connections)
             /odom                                             2872 msgs    : nav_msgs/Odometry                    
             /rosout                                            354 msgs    : rosgraph_msgs/Log                     (10 connections)
             /rosout_agg                                        336 msgs    : rosgraph_msgs/Log                    
             /scan                                              279 msgs    : sensor_msgs/LaserScan                
             /tf                                               3712 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage

```
Now to playback the .bag file and check it, first run the below command in one terminal.
```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
source /opt/ros/kinetic/setup.bash
roscore
```
Now run the following command in new terminal play the storedTurtlebotData.bag file and the results should refect in the listener node.
```
cd <path to repository>/results
rosbag play storedTurtlebotData.bag
```
You can verify by running the below command in new terminal
```
source /opt/ros/kinetic/setup.bash
rostopic echo /cmd_vel_mux/input/navi
```
## RQT Graph
Following is the detailed RQT graph upon successful execution of the nodes.
</p>
<p align="center">
<img src="/readme_images/rosgraph.png">
</p>
</p>
