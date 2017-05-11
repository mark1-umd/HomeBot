/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAHvacAction.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 1, 2017 - Creation
 *
 * @brief Home Automation HVAC Action (set goal temperature; wait for temp to match goal)
 *
 * This class provides a ROS actionlib action that allows a HomeBot ROS node to set a goal
 * temperature for a home; the action server tracks the progress towards the goal, providing
 * feedback to the requestor.  When the temperature reaches the goal, the action has succeeded.
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
#include "homebot/HAHvacAction.hpp"

HAHvacAction::HAHvacAction()
    : homeTempDegF(70.0),
      tempToleranceDegF(0.2),  // Must be at least twice the deltaTempDegF
      deltaTempDegF(0.1),      // Must be no more than 1/2 the tempToleranceDegF
      as(nh, "ha_hvac", boost::bind(&HAHvacAction::actionExecuteCB, this, _1),
         false) {
  as.start();
}

HAHvacAction::~HAHvacAction() {
}

/**
 * @brief ROS action protocol execute callback that acts to reach a specified goal (HVAC mode and temperature)
 * @param [in] goal defined by the HAHvac action
 */
void HAHvacAction::actionExecuteCB(const homebot::HAHvacGoalConstPtr &goal) {
  homebot::HAHvacFeedback feedback;
  homebot::HAHvacResult result;

  // Test for HVAC goal mode validity
  std::string hvacMode;
  if (goal->mode == homebot::HAHvacGoal::COOL)
    hvacMode = "Cool";
  else if (goal->mode == homebot::HAHvacGoal::HEAT)
    hvacMode = "Heat";
  else {
    ROS_WARN_STREAM(
        "HomeBot-HAHvacAction(actionExecuteCB): Aborted; invalid goal mode " << int(goal->mode) << " specified");
    result.tempDegF = homeTempDegF;
    as.setAborted(result);
    return;
  }

  // Test for HVAC goal temperature within range
  if (goal->tempDegF < homebot::HAHvacGoal::MINTEMPDEGF
      || goal->tempDegF > homebot::HAHvacGoal::MAXTEMPDEGF) {
    ROS_WARN_STREAM(
        "HomeBot-HAHvacAction(actionExecuteCB): Aborted; goal temperature outside range " << double(homebot::HAHvacGoal::MINTEMPDEGF) << " to " << double(homebot::HAHvacGoal::MAXTEMPDEGF) << " degrees F");
    result.tempDegF = homeTempDegF;
    as.setAborted(result);
    return;
  }

  // Simulate command to Home Automation system for HVAC temperature change
  ROS_INFO_STREAM(
      "HomeBot-HAHvacAction(actionExecuteCB): Commanding Home Automation system to set " << hvacMode << " mode to " << goal->tempDegF << " degrees F");

  // Simulate the temperature changing in home due to Home Automation HVAC command
  // Establish a ros::rate object to create simulated time...  make deltaTempDegF change every 1 seconds
  ros::Rate delay(10);  // Will cycle at 10 Hz, 1/10 second per deltaTempDegF, so 1 degree/second if delta = .1 deg

  // Should the temperature be rising or falling?  (Assume it will move in direction of goal)
  double tempAdjustRate;
  if (homeTempDegF < goal->tempDegF)
    tempAdjustRate = deltaTempDegF;
  else
    tempAdjustRate = -1 * deltaTempDegF;

  // While the home temperature is outside of the allowed tolerance
  while ((homeTempDegF < (goal->tempDegF - tempToleranceDegF))
      || (homeTempDegF > (goal->tempDegF + tempToleranceDegF))) {
    // Bail out if this action has been preempted or ROS is not running
    if (as.isPreemptRequested() || !ros::ok()) {
      ROS_INFO_STREAM("HomeBot-HAHvacAction(actionExecuteCB): Preempted");
      as.setPreempted();
      return;
    }
    // Adjust the temperature by the deltaTempDegF in the direction of the goal and publish feedback
    homeTempDegF += tempAdjustRate;
    feedback.tempDegF = homeTempDegF;
    ROS_INFO_STREAM(
        "HomeBot-HAHvacAction(actionExecuteCB): Feedback; current temperature is " << feedback.tempDegF << " degrees F");
    as.publishFeedback(feedback);
    delay.sleep();
  }

  // Reached the goal, action completed
  result.tempDegF = homeTempDegF;
  ROS_INFO_STREAM(
      "HomeBot-HAHvacAction(actionExecuteCB): Success; achieved temperature of " << result.tempDegF << " degrees F");
  as.setSucceeded(result);
  return;
}
