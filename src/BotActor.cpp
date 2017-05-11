/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotActor.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 9, 2017 - Creation
 *
 * @brief The HomeBot BotActor is the action server for HomeBot behaviors
 *
 * The HomeBot BotActor receives goals via the ROS actionlib action protocol.  Each goal names a behavior
 * and an number of repetitions for the behavior.  If the behavior exists, and the number of repetitions
 * is within certain parameters, the action server performs the phases of the behavior that causes the
 * behavior activity to be carried out by various parts of the system.
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
#include "homebot/BotActor.hpp"

BotActor::BotActor(Repertoire& pRepertoire, BotOprClients& pOprClients)
    : repertoire(pRepertoire),
      oprClients(pOprClients),
      nh(),
      as(nh, "bot_actor", boost::bind(&BotActor::actionExecuteCB, this, _1),
         false) {
  as.start();
  ROS_INFO_STREAM(
      "HomeBot-BotActor(constructor): Initialized bot_actor action server");
  }

BotActor::~BotActor() {
}

void BotActor::actionExecuteCB(const homebot::HBBehaviorGoalConstPtr &goal) {
  homebot::HBBehaviorFeedback feedback;
  homebot::HBBehaviorResult result;

  // Avoid problems with funny message definitions from ROS and stream processing
  std::string behaviorName = goal->behavior;
  int repsGoal = goal->repetitions;
  ROS_ERROR_STREAM(
      "HomeBot-BotActor(actionExecuteCB): Goal is behavior '" << behaviorName << "' executed " << repsGoal << " time(s)");

  // See if the behavior is in our repertoire
  BotBehavior botBehavior = repertoire.getBehavior(behaviorName);
  if ("" == botBehavior.getName()) {
    ROS_WARN_STREAM(
        "HomeBot-BotActor(actionExecuteCB): Behavior '" << behaviorName << "' not found in repertoire; aborting");
    result.repetitions = 0;
    as.setAborted(result);
    return;
  }

  // Make sure the number of repetitions requested is reasonable
  if (repsGoal < 1 || repsGoal > 10) {
    ROS_WARN_STREAM(
        "HomeBot-BotActor(actionExecuteCB): Behavior '" << behaviorName << "' cannot be repeated " << repsGoal << " times; aborting");
    result.repetitions = 0;
    as.setAborted(result);
    return;
  }

  // Begin the behavior if we haven't been preempted already
  if (!as.isPreemptRequested() && ros::ok()) {
    ROS_INFO_STREAM(
        "HomeBot-BotActor(actionExecuteCB): Beginning behavior '" << behaviorName << "'");
    // Perform the preliminary phase of the behavior using the operation clients
    botBehavior.performPrelim(oprClients);
  } else {
    ROS_INFO_STREAM(
        "HomeBot-BotActor(actionExecuteCB): Behavior '" << behaviorName << "' preempted before preliminary phase");
    as.setPreempted();
    return;
  }

  // Now perform the main behavior the requested number of repetitions
  int repsAttempted, repsPerformed(0);
  for (repsAttempted = 1; repsAttempted <= repsGoal; repsAttempted++) {
    if (!as.isPreemptRequested() && ros::ok()) {
      ROS_ERROR_STREAM(
          "HomeBot-BotActor(actionExecuteCB): Behavior '" << behaviorName << "' main phase, repetition " << repsAttempted);
      botBehavior.performMain(oprClients);
      repsPerformed++;
      feedback.repetitions = repsPerformed;
      as.publishFeedback(feedback);
    } else {
      ROS_ERROR_STREAM(
          "HomeBot-BotActor(actionExecuteCB): Preempt or error occured while executing behavior '" << behaviorName << "' in the main phase, repetition " << repsPerformed);
      break;
    }
  }

  // Regardless of whether the behavior has been pre-empted, if we did the preliminary
  // phase of the behavior, we have to do the post phase of the behavior
  if (ros::ok()) {
    ROS_ERROR_STREAM(
        "HomeBot-BotActor(actionExecuteCB): Behavior '" << behaviorName << "' post phase");
    botBehavior.performPost(oprClients);
  }

  // Did we achieve our goal?
  ROS_ERROR_STREAM(
      "HomeBot-BotActor(actionExecuteCB): Goal check - performed " << repsPerformed << " out of " << repsGoal);
  if (repsPerformed == repsGoal) {
    result.repetitions = repsPerformed;
    as.setSucceeded(result);
    return;
  } else if (as.isPreemptRequested()) {
    as.setPreempted();
    return;
  }

  // If we get here something is confused, so signal an error
  ROS_ERROR_STREAM(
      "HomeBot-BotActor(actionExecuteCB): returning without success or preemption");
  result.repetitions = repsPerformed;
  as.setAborted(result);
  return;
}
