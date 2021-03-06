/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAHvacAction.hpp
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_HAHVACACTION_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_HAHVACACTION_HPP_

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "homebot/HAHvacAction.h"

/** @brief An object to hold the action server for controlling home temperature through the Home Automation system
 */

class HAHvacAction {
 public:
  HAHvacAction();
  virtual ~HAHvacAction();
  void actionExecuteCB(const homebot::HAHvacGoalConstPtr &goal);
 private:
  double homeTempDegF;
  double tempToleranceDegF;
  double deltaTempDegF;
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<homebot::HAHvacAction> as;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_HAHVACACTION_HPP_ */
