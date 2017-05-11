/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HARequests.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 25, 2017 - Creation
 *
 * @brief [Deprecated] Translate Home Automation service requests from the HomeBot system into Home Automation commands
 *
 * HomeBot entities that need services provided by an external Home Automation system
 * that has been interfaced to the HomeBot system use the ROS Action Protocol to request the
 * Home Automation services defined as ROS actions.  This ROS node provides the various
 * action servers required for this purpose.
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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homebot/HAOpenDoorAction.h>

typedef actionlib::SimpleActionServer<homebot::HAOpenDoorAction> ServerType;

/**
 * @brief ROS action execute callback to open/close a door [deprecated; using service now]
 * @param goal object defined by the HAOpenDoor action definition
 * @param as
 */
void execute(const homebot::HAOpenDoorGoalConstPtr& goal, ServerType* as) {
  ROS_INFO_STREAM(
      "Home Automation Open Door command to open door " << goal->door << " sent");
  as->setSucceeded();
}

int main(int argc, char** argv) {
  // Initialize ROS - 3rd argument is ROS node name that will be used for this node (must be unique)
  ros::init(argc, argv, "ha_requests_server");
  ros::NodeHandle n;
  ServerType server(n, "ha_open_door", boost::bind(&execute, _1, &server),
                    false);
  server.start();
  ros::spin();
  return 0;
}
