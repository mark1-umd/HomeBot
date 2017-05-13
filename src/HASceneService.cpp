/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HASceneService.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 28, 2017 - Creation
 *
 * @brief Provides a Home Automation "scene" service (sends HA scene commands)
 *
 * A home may be equipped with one or more scenes operable by a Home Automation system.
 * This service accepts ROS service requests and sends Home Automation commands that
 * will act on the scenes operable by a Home Automation system.
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

#include "homebot/HASceneService.hpp"

/**
 * @brief Constructor for HASceneService; creates an HAScene Service object that must be initialized before it is used
 */
HASceneService::HASceneService() {
}

HASceneService::~HASceneService() {
}

/**
 * @brief Service callback for the Home Automation Scene service - handles service calls
 * @param [in] req data specifying the request details (scene number, action to take)
 * @param [out] rsp data going back to the service requestor (scene number, scene state)
 * @return boolean success or failure of the service call
 */
bool HASceneService::callback(homebot::HAScene::Request& req,
                              homebot::HAScene::Response& rsp) {
  // Validate that the scene number is between 1 and the number of scenes (inclusive)
  if (req.sceneNumber < 1 || req.sceneNumber > sceneState[0]) {
    ROS_WARN_STREAM(
        "HomeBot-HASceneService(callback): Non-existent scene " << int(req.sceneNumber) << " specified for HAScene service action " << int(req.action) << ", no action taken");
    rsp.sceneNumber = req.sceneNumber;
    rsp.state = req.action;
    return false;
  }
  // Validate that the action requested is between the lowest and highest action values
  if (req.action < homebot::HASceneRequest::TURNOFF
      || req.action > homebot::HASceneRequest::STATUS) {
    ROS_WARN_STREAM(
        "HomeBot-HASceneService(callback): Non-existent action " << int(req.action) << " specified for HAScene service on scene " << int(req.sceneNumber) << ", no action taken");
    rsp.sceneNumber = req.sceneNumber;
    rsp.state = req.action;
    return false;
  }
  // Based on the action requested, format a command and send it to the Home
  // Automation system.  Since this is just a demo, no actual commands will be
  // sent, but an ROS log message will be sent
  switch (req.action) {
    case homebot::HASceneRequest::TURNOFF:
      ROS_INFO_STREAM(
          "HomeBot-HASceneService(callback): Sending Close Scene command for scene " << int(req.sceneNumber) << " to Home Automation system");
      sceneState[req.sceneNumber] = homebot::HASceneResponse::OFF;
      break;
    case homebot::HASceneRequest::TURNON:
      ROS_INFO_STREAM(
          "HomeBot-HASceneService(callback): Sending Open Scene command for scene " << int(req.sceneNumber) << " to Home Automation system");
      sceneState[req.sceneNumber] = homebot::HASceneResponse::ON;
      break;
    case homebot::HASceneRequest::STATUS:
      ROS_INFO_STREAM(
          "HomeBot-HASceneService(callback): Sending Scene Status command for scene " << int(req.sceneNumber) << " to Home Automation system");
      break;
  }
  rsp.sceneNumber = req.sceneNumber;
  rsp.state = sceneState[rsp.sceneNumber];
  return true;
}

/**
 * @brief Called to start the actual service; advertises the service and establishes callback
 * @param numberOfScenes used to size the number of scene states to track
 */
void HASceneService::init(int numberOfScenes) {
  ROS_INFO_STREAM(
      "HomeBot-HASceneService(init): HASceneService initializing; " << numberOfScenes << " scenes to initialize.");
  // Store the number of scenes in the first element of the scene state vector
  sceneState.push_back(numberOfScenes);
  for (int d = 1; d <= numberOfScenes; d++) {
    // Initialize all of the scene states to closed
    sceneState.push_back(homebot::HASceneResponse::OFF);
    ROS_INFO_STREAM(
        "HomeBot-HASceneService(init): Scene " << d << " initialized to state " << sceneState[d]);
  }
  // Set the service server object using the node handle's advertiseService method,
  // the service name, and the callback method from this object
  ss = nh.advertiseService("ha_scene", &HASceneService::callback, this);
}
