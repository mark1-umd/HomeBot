/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HARequestServer_test.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 27, 2017 - Creation
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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "homebot/HADoor.h"
#include "homebot/HAScene.h"
#include "homebot/HAShade.h"

// Set up the HARequestServer HADoor service test using the GoogleTest macros
// This test assumes that the HARequestServer has been initialized with 5 doors
// (argument -doors 5)
TEST (HARequestServer, HADoorService) {
  // Set the number of doors in the server to be tested
  int doorCount = 5;

  // Set up as a service client
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<homebot::HADoor>("ha_door");

  // Test the availability of the service
  bool serviceAvailable(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(serviceAvailable);

  // Now test service requests and responses
  bool success;
  homebot::HADoor srv;

  // Should not be door number 0 or a door greater than doorCount
  srv.request.action = homebot::HADoorRequest::STATUS;
  srv.request.doorNumber = 0;
  success = client.call(srv);
  EXPECT_FALSE(success);
  srv.request.doorNumber = doorCount + 1;
  success = client.call(srv);
  EXPECT_FALSE(success);

  // The doors should all initialize to closed
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::CLOSED, srv.response.state);
  }

  // Should be able to open all of the doors
  srv.request.action = homebot::HADoorRequest::OPEN;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::OPENED, srv.response.state);
  }

  // Now all of the doors should be open
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::OPENED, srv.response.state);
  }

  // Should be able to close all of the doors
  srv.request.action = homebot::HADoorRequest::CLOSE;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::CLOSED, srv.response.state);
  }

  // Now all of the doors should be closed
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (doorCount + 1); i++) {
    srv.request.doorNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HADoorResponse::CLOSED, srv.response.state);
  }
}
/*******************************************************************************/

// Set up the HARequestServer HAScene service test using the GoogleTest macros
// This test assumes that the HARequestServer has been initialized with 15 scenes
// (argument -scenes 15)
TEST (HARequestServer, HASceneService) {
  // Set the number of scenes in the server to be tested
  int sceneCount = 15;

  // Set up as a service client
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<homebot::HAScene>("ha_scene");

  // Test the availability of the service
  bool serviceAvailable(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(serviceAvailable);

  // Now test service requests and responses
  bool success;
  homebot::HAScene srv;

  // Should not be scene number 0 or a scene greater than sceneCount
  srv.request.action = homebot::HASceneRequest::STATUS;
  srv.request.sceneNumber = 0;
  success = client.call(srv);
  EXPECT_FALSE(success);
  srv.request.sceneNumber = sceneCount + 1;
  success = client.call(srv);
  EXPECT_FALSE(success);

  // The scenes should all initialize to off
  srv.request.action = homebot::HASceneRequest::STATUS;
  for (int i = 1; i < (sceneCount + 1); i++) {
    srv.request.sceneNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HASceneResponse::OFF, srv.response.state);
  }

  // Should be able to turn on all of the scenes
  srv.request.action = homebot::HASceneRequest::TURNON;
  for (int i = 1; i < (sceneCount + 1); i++) {
    srv.request.sceneNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HASceneResponse::ON, srv.response.state);
  }

  // Now all of the scenes should be on
  srv.request.action = homebot::HASceneRequest::STATUS;
  for (int i = 1; i < (sceneCount + 1); i++) {
    srv.request.sceneNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HASceneResponse::ON, srv.response.state);
  }

  // Should be able to turn off all of the scenes
  srv.request.action = homebot::HASceneRequest::TURNOFF;
  for (int i = 1; i < (sceneCount + 1); i++) {
    srv.request.sceneNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HASceneResponse::OFF, srv.response.state);
  }

  // Now all of the scenes should be turned off
  srv.request.action = homebot::HASceneRequest::STATUS;
  for (int i = 1; i < (sceneCount + 1); i++) {
    srv.request.sceneNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HASceneResponse::OFF, srv.response.state);
  }
}
/*******************************************************************************/

// Set up the HARequestServer HAShade service test using the GoogleTest macros
// This test assumes that the HARequestServer has been initialized with 8 shades
// (argument -shades 8)
TEST (HARequestServer, HAShadeService) {
  // Set the number of shades in the server to be tested
  int shadeCount = 8;

  // Set up as a service client
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<homebot::HAShade>("ha_shade");

  // Test the availability of the service
  bool serviceAvailable(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(serviceAvailable);

  // Now test service requests and responses
  bool success;
  homebot::HAShade srv;

  // Should not be shade number 0 or a shade greater than shadeCount
  srv.request.action = homebot::HAShadeRequest::STATUS;
  srv.request.shadeNumber = 0;
  success = client.call(srv);
  EXPECT_FALSE(success);
  srv.request.shadeNumber = shadeCount + 1;
  success = client.call(srv);
  EXPECT_FALSE(success);

  // The shades should all initialize to raised
  srv.request.action = homebot::HAShadeRequest::STATUS;
  for (int i = 1; i < (shadeCount + 1); i++) {
    srv.request.shadeNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HAShadeResponse::RAISED, srv.response.state);
  }

  // Should be able to lower all of the shades
  srv.request.action = homebot::HAShadeRequest::LOWER;
  for (int i = 1; i < (shadeCount + 1); i++) {
    srv.request.shadeNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HAShadeResponse::LOWERED, srv.response.state);
  }

  // Now all of the shades should be lowered
  srv.request.action = homebot::HAShadeRequest::STATUS;
  for (int i = 1; i < (shadeCount + 1); i++) {
    srv.request.shadeNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HAShadeResponse::LOWERED, srv.response.state);
  }

  // Should be able to close all of the shades
  srv.request.action = homebot::HADoorRequest::CLOSE;
  for (int i = 1; i < (shadeCount + 1); i++) {
    srv.request.shadeNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HAShadeResponse::RAISED, srv.response.state);
  }

  // Now all of the shades should be raised
  srv.request.action = homebot::HADoorRequest::STATUS;
  for (int i = 1; i < (shadeCount + 1); i++) {
    srv.request.shadeNumber = i;
    success = client.call(srv);
    EXPECT_TRUE(success);
    EXPECT_EQ(homebot::HAShadeResponse::RAISED, srv.response.state);
  }
}
/*******************************************************************************/

/*
 * @brief Initialize ROS and run all of the tests defined in the test macros
 */
int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "HARequestServer_test");

  // Run all of the Google Test defined tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
