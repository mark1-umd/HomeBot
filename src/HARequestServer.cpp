/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HARequestServer.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 27, 2017 - Creation
 *
 * @brief Main function for the Home Automation Request Server - a ROS node
 *
 * The HomeBot system interfaces with Home Automation systems.  This ROS node
 * provides ROS services used to request HA commands by other ROS nodes.  In
 * a full implementation, the services would generate the required Home Automation
 * system commands (as HTTP GET/POST, SOAP, RS-232, or other) required to provide
 * the services requested.  In this demonstration implementation, the services do
 * not command an actual Home Automation system.
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
#include "HADoorService.hpp"
#include "HASceneService.hpp"
#include "HAShadeService.hpp"

/**
 * The main function handles the ROS node lifespan
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "HARequestServer");

  // Process command line arguments left after ROS strips off remapping arguments
  int doorCount = 0;
  int sceneCount = 0;
  int shadeCount = 0;

  for (int i = 1; i < argc; i++) {
    if (i + 1 != argc) {
      // If we match the parameter
      if (strcmp(argv[i], "-doors") == 0) {
        // Grab the number of doors for the HADoorService; should be an integer
        doorCount = atoi(argv[i + 1]);
        // Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-scenes") == 0) {
        // Grab the number of scenes for the HASceneService; should be an integer
        sceneCount = atoi(argv[i + 1]);
        // Skip the parameter value
        i++;
      } else if (strcmp(argv[i], "-shades") == 0) {
        // Grab the number of shades for the HAShadeService; should be an integer
        shadeCount = atoi(argv[i + 1]);
        // Skip the parameter value
        i++;
      }
    }
  }
  // ROS startup happens when first node handle is created; shutdown should occur
  // automatically when last node handle is destroyed or SIGINT captures Ctrl-C
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Home Automation Request server has started");

  // Start the Home Automation Door Service
  HADoorService hADoorServer;
  hADoorServer.init(doorCount);

  // Start the Home Automation Scene Service
  HASceneService hASceneServer;
  hASceneServer.init(sceneCount);

  // Start the Home Automation Door Service
  HAShadeService hAShadeServer;
  hAShadeServer.init(shadeCount);

  // spin so that callbacks for services can be processed
  ros::spin();

  return 0;
}
