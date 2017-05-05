/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotActor.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief <brief description>
 *
 * <details>
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
#include "BotActor.hpp"

BotActor::BotActor(const Repertoire& pRepertoire)
    : repertoire(pRepertoire),
      nh(),
      acBotMoveBase("move_base", true),
      as(nh, "bot_actor", boost::bind(&BotActor::actionExecuteCB, this, _1),
         false) {
  scHADoorAffect = nh.serviceClient<homebot::HADoor>("ha_door");
  scHASceneAffect = nh.serviceClient<homebot::HAScene>("ha_scene");
  scHAShadeAffect = nh.serviceClient<homebot::HAShade>("ha_shade");
  while (!acBotMoveBase.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to start");
  }
  while (!scHADoorAffect.waitForExistence(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the ha_door service to start");
  }
  while (!scHASceneAffect.waitForExistence(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the ha_scene service to start");
  }
  while (!scHAShadeAffect.waitForExistence(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the ha_shade service to start");
  }
}

BotActor::~BotActor() {
    // TODO(Mark Jenkins): Auto-generated destructor stub
}

void BotActor::actionExecuteCB(const homebot::HBBehaviorGoalConstPtr &goal) {
  homebot::HBBehaviorFeedback feedback;
  homebot::HBBehaviorResult result;

  // See if the goal behavior exists
  std::string behaviorName;  // this should actually come from the goal directly once I update action definition to be a string not integer
  BotBehavior botBehavior;
  if (!repertoire.findBehavior(behaviorName, botBehavior)) {
    ROS_WARN_STREAM(
        "BotBehavior " << behaviorName << " not found in repertoire; aborting");
    result.repetitions = 0;
    as.setAborted(result);
    return;
  }

  // Make sure the number of repetitions requested is reasonable
  if (goal->repetitions < 1 || goal->repetitions > 10) {
    ROS_WARN_STREAM(
        "BotBehavior " << behaviorName << " cannot be repeated " << goal->repetitions << " times; aborting");
    result.repetitions = 0;
    as.setAborted(result);
  }

  // Begin the behavior if we haven't been preempted already
  if (!as.isPreemptRequested() && ros::ok()) {
    ROS_INFO_STREAM("BotBehavior beginning " << behaviorName);
    botBehavior.beginning();
  } else {
    ROS_INFO_STREAM(
        "BotBehavior " << behaviorName << " preempted before beginning");
    as.setPreempted();
    return;
  }

  // Now perform the main behavior the requested number of repetitions
  int repetitions;
  for (repetitions = 1; repetitions <= goal->repetitions; repetitions++) {
    if (!as.isPreemptRequested && ros::ok()) {
      ROS_INFO_STREAM(
          "BotBehavior main " << behaviorName << " " << repetitions);
      botBehavior.main();
      feedback.repetition = repetitions;
      as.publishFeedback(feedback);
    } else {
      repetitions--;
      break;
    }
  }

  // Regardless of whether the behavior has been pre-empted,
  // if we did the behavior beginning, we have to do the behavior finishing
  if (ros::ok()) {
    ROS_INFO_STREAM("BotBehavior " << behaviorName << " finishing");
    botBehavior.finishing();
  }

  // Did we achieve our goal?
  if (repetitions == goal->repetitions) {
    result.repetitions = repetitions;
    as.setSucceeded(result);
  } else if (as.isPreemptRequested()) {
    as.setPreempted();
    return;
  }

  // If we get here something is confused, so signal an error
  ROS_ERROR_STREAM("BotBehavior terminating without success or preemption");
  return;
}
