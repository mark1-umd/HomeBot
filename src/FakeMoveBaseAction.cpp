/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file FakeMoveBaseAction.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 10, 2017 - Creation
 *
 * @brief Fake MoveBase provides an action server that simulates a navigation stack move_base action server
 *
 * This class provides a ROS actionlib action server that allows an action client to set a goal
 * pose; the action server tracks the progress towards the goal, providing feedback to the
 * requestor.  When the simulated goal is reached, the action has succeeded.
 *
 * *
 * * BSD 3-Clause License
 *
 * Copyright (c) 2017, Mark Jenkins
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
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
 */
#include "homebot/FakeMoveBaseAction.hpp"

FakeMoveBaseAction::FakeMoveBaseAction()
    : fbFreq(10),  // feedback frequency in Hz; also controls time for simulation
      baseVel(1.0),       // base velocity in units (assumed meters) per second
      posX(0.0),          // X coordinate of current base location; starts at 0
      posY(0.0),          // Y coordinate of current base location; starts at 0
      posZ(0.0),          // Z coordinate of current base location; starts at 0
      as(nh, "move_base",
         boost::bind(&FakeMoveBaseAction::actionExecuteCB, this, _1),
         false) {
  as.start();
}

FakeMoveBaseAction::~FakeMoveBaseAction() {
}

/**
 * @brief Calculates the distance from one 2D point to another 2D point
 * @param [in] x1 X coordinate of first point
 * @param [in] y1 Y coordinate of first point
 * @param [in] x2 X coordinate of second point
 * @param [in] y2 Y coordinate of second point
 * @return distance from first point to second point
 */
double FakeMoveBaseAction::distance(double x1, double y1, double x2,
                                    double y2) {
  return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

/**
 * @brief Executes whenever the action server receives a goal from an action client
 * @param goal
 */
void FakeMoveBaseAction::actionExecuteCB(
    const move_base_msgs::MoveBaseGoalConstPtr &goal) {
  move_base_msgs::MoveBaseFeedback feedback;
  move_base_msgs::MoveBaseResult result;

  double goalX = goal->target_pose.pose.position.x;
  double goalY = goal->target_pose.pose.position.y;

  // Simulate command to Home Automation system for HVAC temperature change
  ROS_INFO_STREAM(
      "FakeMoveBase(actionExecuteCB): moving from (" << posX << ", " << posY << ") to (" << goalX << ", " << goalY << ") at " << baseVel << " m/s");

  // Simulate the move at a rate of fbFreq Hz using a ROS rate object
  ros::Rate delay(fbFreq);

  // Fix our rate of change in position at the base velocity / feedback frequency
  // (feedback frequency controls delta time for the simulation)
  double deltaP = baseVel / fbFreq;

  // While the base is not at the goal location [(X,Y) only] plan our next move
  while ((posX != goalX) && (posY != goalY)) {
    ROS_DEBUG_STREAM(
        "FakeMoveBase(actionExecuteCB): Not at goal; planning move");
    // Bail out if this action has been preempted or ROS is not running
    if (as.isPreemptRequested() || !ros::ok()) {
      ROS_INFO_STREAM("FakeMoveBase(actionExecuteCB): Preempted");
      as.setPreempted();
      return;
    }
    // Adjust our position by deltaP  or the actual distance, whichever is less
    double distanceToGoal = distance(posX, posY, goalX, goalY);
    ROS_DEBUG_STREAM(
        "FakeMoveBase(actionExecuteCB): Distance to goal is " << distanceToGoal << " meters");
    if (distanceToGoal < deltaP) {
      ROS_DEBUG_STREAM(
          "FakeMoveBase(actionExecuteCB): Less than " << deltaP << " meters, we are going to be there next!");
      posX = goalX;
      posY = goalY;
    } else {
      double theta = atan2((goalY - posY), (goalX - posX));
      double deltaX = cos(theta) * deltaP;
      double deltaY = sin(theta) * deltaP;
      posX += deltaX;
      posY += deltaY;
    }
    // Prepare our feedback based on our updated position
    feedback.base_position.pose.position.x = posX;
    feedback.base_position.pose.position.y = posY;
    feedback.base_position.pose.position.z = posZ;
    feedback.base_position.pose.orientation.x = orientX;
    feedback.base_position.pose.orientation.y = orientY;
    feedback.base_position.pose.orientation.z = orientZ;
    feedback.base_position.pose.orientation.w = orientW;
    // Simulate delay for motion at the rate of baseVel meters/second
    delay.sleep();
    ROS_INFO_STREAM(
        "FakeMoveBase(actionExecuteCB): Feedback; moved to (" << posX << ", " << posY << ")");
    as.publishFeedback(feedback);
  }

  // Reached the goal, action completed
  ROS_INFO_STREAM(
      "FakeMoveBase(actionExecuteCB): Reached goal at (" << posX << ", " << posY << ")");
  as.setSucceeded();
  return;
}
