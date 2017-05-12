/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAHvacActionServer.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 2, 2017 - Creation
 *
 * @brief ROS node that contains the Home Automation HVAC Action Server
 *
 * The Home Automation HVAC Action Server allows HomeBot ROS nodes to set action goals for the
 * Home Automation system's temperature control capability.
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
#include "homebot/FakeMoveBaseAction.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "hb_fake_move_base");

  // Process command line arguments left after ROS strips off remapping arguments
  double feedbackFrequency = 10.0;
  double baseVelocity = 1.0;

  for (int i = 1; i < argc; i++) {
    if (i + 1 != argc) {
      // If we match the parameter
      if (strcmp(argv[i], "-freq") == 0) {
        // Grab the number of doors for the HADoorService; should be an integer
        feedbackFrequency = atof(argv[i + 1]);
        // Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-vel") == 0) {
        // Grab the number of scenes for the HASceneService; should be an integer
        baseVelocity = atof(argv[i + 1]);
        // Skip the parameter value
        i++;
      }
    }
  }

  FakeMoveBaseAction fakeMoveBaseAction(feedbackFrequency, baseVelocity);
  // ROS startup happens when first node handle is created; shutdown should occur
  // automatically when last node handle is destroyed or SIGINT captures Ctrl-C
  ROS_INFO_STREAM(
      "HomeBot-FakeMoveBaseServer(main): Fake Move Base action server has started");

  ros::spin();

  return 0;
}
