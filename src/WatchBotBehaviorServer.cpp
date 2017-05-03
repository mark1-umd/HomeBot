/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file WatchBotBehaviorServer.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 2, 2017 - Creation
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

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "homebot/HBBehaviorAction.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void actionExecuteCB(const homebot::HBBehaviorGoalConstPtr &goal) {
  move_base_msgs::MOveBaseGoal mbgoal;

  //goal.taret_pose.header.frame_id = "map"
  mbgoal.target_pose.header.stamp = ros::Time::now();

  mbgoal.target_pose.pose.position.x = 0.0;
  mbgoal.target_pose.pose.position.y = 0.0;
  mbgoal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO_STREAM("Sending goal");
  mbac.sendGoal(mbgoal);

  mbac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Reached goal");
  else
    ROS_INFO_STREAM("Failed to reach goal");

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wb_behavior_server");
  ros::NodeHandle nh;

  MoveBaseClient mbac("move_base", true);
  while (!mbac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  actionlib::SimpleActionServer<homebot::HBBehaviorAction> hbas(
      nh, "hb_behavior", boost::bind(&actionExecuteCB), false);

  ros::spin();

  return 0;
}
