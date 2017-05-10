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
    : baseVel(0.1),
      posX(0.0),
      posY(0.0),
      posZ(0.0),
      as(nh, "move_base",
         boost::bind(&FakeMoveBaseAction::actionExecuteCB, this, _1),
         false) {
  as.start();
}

FakeMoveBaseAction::~FakeMoveBaseAction() {
}

double FakeMoveBaseAction::distance(double x1, double y1, double x2,
                                    double y2) {
  return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

void FakeMoveBaseAction::actionExecuteCB(
    const move_base_msgs::MoveBaseGoalConstPtr &goal) {
  move_base_msgs::MoveBaseFeedback feedback;
  move_base_msgs::MoveBaseResult result;

  double goalX = goal->target_pose.pose.position.x;
  double goalY = goal->target_pose.pose.position.y;

  // Simulate command to Home Automation system for HVAC temperature change
  ROS_INFO_STREAM(
      "FakeMoveBase(actionExecuteCB): moving from (" << posX << ", " << posY << ") to (" << goalX << ", " << goalY << " at " << baseVel << " m/s");

  // Simulate the move at a rate of 10 HZ using a ROS rate object
  ros::Rate delay(fbFreq);  // Will cycle at 10 Hz, 1/10 second per deltaTempDegF, so 1 degree/second if delta = .1 deg

  // Fix our rate of change in position at the base velocity / feedback frequency
  double deltaP = baseVel / fbFreq;

  // While the home temperature is outside of the allowed tolerance
  while ((posX != goalX) && (posY != goalY)) {
    // Bail out if this action has been preempted or ROS is not running
    if (as.isPreemptRequested() || !ros::ok()) {
      ROS_INFO_STREAM("FakeMoveBase(actionExecuteCB): Preempted");
      as.setPreempted();
      return;
    }
    // Adjust our position by deltaP  or the actual distance, whichever is less
    double distanceToGoal = distance(posX, posY, goalX, goalY);
    if (distanceToGoal < deltaP) {
      posX = goalX;
      posY = goalY;
    } else {
      double theta = asin((goalY - posY) / distanceToGoal);
      double deltaX = cos(theta) / deltaP;
      double deltaY = sin(theta) / deltaP;
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
