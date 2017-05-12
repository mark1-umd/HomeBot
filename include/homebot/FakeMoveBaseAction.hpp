/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file FakeMoveBaseAction.hpp
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_FAKEMOVEBASEACTION_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_FAKEMOVEBASEACTION_HPP_

#include <cmath>
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/server/simple_action_server.h"

/** @brief An object to hold the action server for faking move_base actions in the HomeBot demo
 */

class FakeMoveBaseAction {
 public:
  FakeMoveBaseAction();
  FakeMoveBaseAction(double pFBFreq, double pBaseVel);
  virtual ~FakeMoveBaseAction();
  void actionExecuteCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
 private:
  double fbFreq;
  double baseVel;
  double posX;
  double posY;
  double posZ;
  double orientX;
  double orientY;
  double orientZ;
  double orientW;
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as;
  double distance(double x1, double y1, double x2, double y2);
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_FAKEMOVEBASEACTION_HPP_ */
