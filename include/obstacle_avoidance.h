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
 *  @file    obstacle_avoidance.h
 *
 *  @author  Hrishikesh Tawade
 *
 *  @copyright BSD License
 *
 *  @brief Library for obstacle_avoidance ROS node
 *
 *  @section DESCRIPTION
 *
 *  This library contains the declarations for the class Robot
 */


#ifndef INCLUDE_OBSTACLE_AVOIDANCE_H_
#define INCLUDE_OBSTACLE_AVOIDANCE_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief  Declarations for class Robot .
 */
class Robot {
 private:
  // Create a ROS node handle
  ros::NodeHandle n;
  // Creating ROS publisher
  ros::Publisher pub;
  // Creating ROS subscriber
  ros::Subscriber sub;
  // Variable to detect collision
  bool obstacleDetected;
  // Variable for velocities
  geometry_msgs::Twist vel;
  // Variable for linear and turning speed
  float linearSpeed;
  float turningSpeed;

 public:
  /**
   * @brief   Constructor for Robot Class
   */
  Robot();
  /**
   * @brief  Destructor forthe Robot Class.
   */
  ~Robot();

  /**
   * @brief  Tells the robot if there is obstacle nearby
   *
   * @return true if obstacle detected
   *         false if obstacle not detected
   */
  bool checkObstacle();

  /**
   * @brief Sets the speeds of the robot
   *
   * @param straight Robot's straight direction speed
   * @param turn Robot's turning speed
   *
   * @return     Nothing
   */
  void setSpeeds(float straight, float turn);

  /**
   * @brief      Starts the robot
   */
  void startRobot();

  /**
   * @brief Function to provide laser scanner readings
   *
   * @param laserData Array of laser scanner points
   */
  void laserScannerCallback(const sensor_msgs::LaserScan::ConstPtr& laserData);
};

#endif  // INCLUDE_OBSTACLE_AVOIDANCE_H_

