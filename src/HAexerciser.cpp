/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAexerciser.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 25, 2017 - Creation
 *
 * @brief HA exerciser makes Home Automation service action requests to exercise Home Automation translation actions
 *
 * HomeBot entities that need services provided by an external Home Automation system
 * that has been interfaced to the HomeBot system use the ROS Action Protocol to request the
 * Home Automation services defined as ROS actions.  This ROS node exercises the various
 * action servers provided for this purpose.
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

#include <actionlib/client/simple_action_client.h>
#include <homebot/HAOpenDoorAction.h>

typedef actionlib::SimpleActionClient<homebot::HAOpenDoorAction> Client;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ha_action_exerciser");
  Client client("ha_open_door", true);  // true -> don't need ros::spin()
  client.waitForServer();
  homebot::HAOpenDoorGoal goal;
  // Fill in goal here
  goal.door = 3;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout << "Yay! HA Request to open door number " << goal.door
              << " succeeded" << std::endl;
  std::cout << "Current State: " << client.getState().toString().c_str()
            << std::endl;
  return 0;
}
