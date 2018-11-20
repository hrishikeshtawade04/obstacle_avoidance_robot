/**
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, Hrishikesh Tawade
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  @copyright (c) BSD
 *
 *  @file    walker.cpp
 *
 *  @author  Hrishikesh Tawade
 *
 *  @copyright BSD License
 *
 *  @brief ROS obstace avoidance robot node
 *
 *  @section DESCRIPTION
 *
 *  This program will run the obstacle avoidance behaviour for
 *  turtlebot
 */

#include <iostream>
#include "../include/obstacle_avoidance.h"

/**
 * @brief  Constructs the object of the class Robot.
 */
Robot::Robot() {
  ROS_INFO("Setting turtlebot cofiguration...");
  // Set some parameters
  setSpeeds(0.5, 0.5);
  // Setting initial obstacle detection value
  obstacleDetected = false;
  // Publish the velocity to turtlebot
  pub = n.advertise < geometry_msgs::Twist > ("/cmd_vel_mux/input/navi", 1000);
  // Subcribe to the /scan topic and use the laserScannerCallback method
  sub = n.subscribe < sensor_msgs::LaserScan
      > ("/scan", 50, &Robot::laserScannerCallback, this);
}

/**
 * @brief  Destroys the object of class Robot.
 */
Robot::~Robot() {
}

/**
 * @brief   Detects obstacles using laser scanner data and notifies robot
 *
 * @param  laserData  Message from /scan topic
 */
void Robot::laserScannerCallback(
    const sensor_msgs::LaserScan::ConstPtr& laserData) {
  // Checking laser data and reporting obstacle found if object in 1m range
  for (auto data : laserData->ranges) {
    if (data < 1.0) {
      obstacleDetected = true;
      return;
    }
  }
  obstacleDetected = false;
  return;
}

/**
 * @brief      Returns the collision flag
 *
 * @return     boolean value for the collision flag
 */
bool Robot::checkObstacle() {
  if (obstacleDetected == false) {
    // Tells user that robot
    ROS_INFO("No obstacle detected. Continuing on straight path");
  } else {
    // Warning user that obstacle is encountered
    ROS_WARN("Obstacle Detected. Changing direction!");
  }
  return obstacleDetected;
}

/**
 * @brief      Sets speeds of the robot
 *
 * @return     nothing
 */
void Robot::setSpeeds(float straight, float turn) {
  // Setting speed to go straight.
  linearSpeed = straight;
  // Setting speed to turn.
  turningSpeed = turn;
}

/**
 * @brief      Runs the robot
 */
void Robot::startRobot() {
  // Setting publishing rate
  ros::Rate loop_rate(10);
  // Keeps running till ROS is ok
  while (ros::ok()) {
    // Checking obstacle
    if (checkObstacle()) {
      // Stopping the robot and turning it
      vel.linear.x = 0.0;
      vel.angular.z = turningSpeed;
    } else {
      // Stopping turning and reseting back to straight path
      vel.angular.z = 0.0;
      vel.linear.x = linearSpeed;
    }
    // Publish the velocity message for gazebo
    pub.publish(vel);
    ros::spinOnce();
    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();
  }
}

