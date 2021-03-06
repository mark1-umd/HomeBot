/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAClients.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 10, 2017 - Creation
 *
 * @brief Holds a set of ROS service and action clients necessary for Home Automation activities
 *
 * The HomeBot HA Clients are the action and service clients necessary for the Home Automation
 * system to act in the HomeBot system. In order for the activities to make use of services
 * and actions, clients for the service and actions have to be available.  Rather than
 * continuously start and stop these clients, this Class provides a way to start them up
 * when a Home Automation Client ROS node initializes, and to hold the client objects so that
 * they can be used by other methods/functions.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_HACLIENTS_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_HACLIENTS_HPP_

#include "ros/ros.h"
#include "homebot/HBBehaviorAction.h"
#include "actionlib/client/simple_action_client.h"

/** @brief An object to hold ROS action/service clients needed to perform Home Automation activities in the HomeBot system
 */

class HAClients {
  friend class HADemoService;
 public:
  HAClients();
  virtual ~HAClients();
 private:
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<homebot::HBBehaviorAction> acHBBehavior;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_HACLIENTS_HPP_ */
