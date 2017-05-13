/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HADoorService.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 27, 2017 - Creation
 *
 * @brief Provides a Home Automation "door" service (sends HA door commands)
 *
 * A home may be equipped with one or more doors operable by a Home Automation system.
 * This service accepts ROS service requests and sends Home Automation commands that
 * will act on the doors operable by a Home Automation system.
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

#include "homebot/HADoorService.hpp"

/**
 * @brief Constructor for HADoorService; creates an object for the HADoor Service that must be initialized before it can be used
 */
HADoorService::HADoorService() {
}

HADoorService::~HADoorService() {
}

/**
 * @brief Service callback for the Home Automation Door service - handles service calls
 * @param [in] req data specifying the request details (door number, action to take)
 * @param [out] rsp data going back to the service requestor (door number, door state)
 * @return boolean success or failure of the service call
 */
bool HADoorService::callback(homebot::HADoor::Request& req,
                             homebot::HADoor::Response& rsp) {
  // Validate that the door number is between 1 and the number of doors (inclusive)
  if (req.doorNumber < 1 || req.doorNumber > doorState[0]) {
    ROS_WARN_STREAM(
        "HomeBot-HADoorService(callback): Non-existent door " << int(req.doorNumber) << " specified for HADoor service action " << int(req.action) << ", no action taken");
    rsp.doorNumber = req.doorNumber;
    rsp.state = req.action;
    return false;
  }
  // Validate that the action requested is between the lowest and highest action values
  if (req.action < homebot::HADoorRequest::CLOSE
      || req.action > homebot::HADoorRequest::STATUS) {
    ROS_WARN_STREAM(
        "HomeBot-HADoorService(callback): Non-existent action " << int(req.action) << " specified for HADoor service on door " << int(req.doorNumber) << ", no action taken");
    rsp.doorNumber = req.doorNumber;
    rsp.state = req.action;
    return false;
  }
  // Based on the action requested, format a command and send it to the Home
  // Automation system.  Since this is just a demo, no actual commands will be
  // sent, but an ROS log message will be sent
  switch (req.action) {
    case homebot::HADoorRequest::CLOSE:
      ROS_INFO_STREAM(
          "HomeBot-HADoorService(callback): Sending Close Door command for door " << int(req.doorNumber) << " to Home Automation system");
      doorState[req.doorNumber] = homebot::HADoorResponse::CLOSED;
      break;
    case homebot::HADoorRequest::OPEN:
      ROS_INFO_STREAM(
          "HomeBot-HADoorService(callback): Sending Open Door command for door " << int(req.doorNumber) << " to Home Automation system");
      doorState[req.doorNumber] = homebot::HADoorResponse::OPENED;
      break;
    case homebot::HADoorRequest::STATUS:
      ROS_INFO_STREAM(
          "HomeBot-HADoorService(callback): Sending Door Status command for door " << int(req.doorNumber) << " to Home Automation system");
      break;
  }
  rsp.doorNumber = req.doorNumber;
  rsp.state = doorState[rsp.doorNumber];
  return true;
}

/**
 * @brief Called to start the actual service; advertises the service and establishes callback
 * @param numberOfDoors used to size the number of door states to track
 */
void HADoorService::init(int numberOfDoors) {
  ROS_INFO_STREAM(
      "HomeBot-HADoorService(init): HADoorService initializing; " << numberOfDoors << " doors to initialize.");
  // Store the number of doors in the first element of the door state vector
  doorState.push_back(numberOfDoors);
  for (int d = 1; d <= numberOfDoors; d++) {
    // Initialize all of the door states to closed
    doorState.push_back(homebot::HADoorResponse::CLOSED);
    ROS_INFO_STREAM(
        "HomeBot-HADoorService(init): Door " << d << " initialized to state " << doorState[d]);
  }
  // Set the service server object using the node handle's advertiseService method,
  // the service name, and the callback method from this object
  ss = nh.advertiseService("ha_door", &HADoorService::callback, this);
}
