/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HAShadeService.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 28, 2017 - Creation
 *
 * @brief Provides a Home Automation "shade" service (sends HA shade commands)
 *
 * A home may be equipped with one or more shades operable by a Home Automation system.
 * This service accepts ROS service requests and sends Home Automation commands that
 * will act on the shades operable by a Home Automation system.
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

#include "homebot/HAShadeService.hpp"

HAShadeService::HAShadeService() {
  // TODO(Mark Jenkins): Auto-generated constructor stub

}

HAShadeService::~HAShadeService() {
  // TODO(Mark Jenkins): Auto-generated destructor stub

}

/**
 * @brief Service callback for the Home Automation Shade service - handles service calls
 * @param [in] req data specifying the request details (shade number, action to take)
 * @param [out] rsp data going back to the service requestor (shade number, shade state)
 * @return boolean success or failure of the service call
 */
bool HAShadeService::callback(homebot::HAShade::Request& req,
                              homebot::HAShade::Response& rsp) {
  // Validate that the shade number is between 1 and the number of shades (inclusive)
  if (req.shadeNumber < 1 || req.shadeNumber > shadeState[0]) {
    ROS_WARN_STREAM(
        "Non-existent shade " << int(req.shadeNumber) << " specified for HAShade service action " << int(req.action) << ", no action taken");
    rsp.shadeNumber = req.shadeNumber;
    rsp.state = req.action;
    return false;
  }
  // Validate that the action requested is between the lowest and highest action values
  if (req.action < homebot::HAShadeRequest::RAISE
      || req.action > homebot::HAShadeRequest::STATUS) {
    ROS_WARN_STREAM(
        "Non-existent action " << int(req.action) << " specified for HAShade service on shade " << int(req.shadeNumber) << ", no action taken");
    rsp.shadeNumber = req.shadeNumber;
    rsp.state = req.action;
    return false;
  }
  // Based on the action requested, format a command and send it to the Home
  // Automation system.  Since this is just a demo, no actual commands will be
  // sent, but an ROS log message will be sent
  switch (req.action) {
    case homebot::HAShadeRequest::RAISE:
      ROS_INFO_STREAM(
          "Sending Close Shade command for shade " << int(req.shadeNumber) << " to Home Automation system");
      shadeState[req.shadeNumber] = homebot::HAShadeResponse::RAISED;
      break;
    case homebot::HAShadeRequest::LOWER:
      ROS_INFO_STREAM(
          "Sending Open Shade command for shade " << int(req.shadeNumber) << " to Home Automation system");
      shadeState[req.shadeNumber] = homebot::HAShadeResponse::LOWERED;
      break;
    case homebot::HAShadeRequest::STATUS:
      ROS_INFO_STREAM(
          "Sending Shade Status command for shade " << int(req.shadeNumber) << " to Home Automation system");
      break;
  }
  rsp.shadeNumber = req.shadeNumber;
  rsp.state = shadeState[rsp.shadeNumber];
  return true;
}

/**
 * @brief Called to start the actual service; advertises the service and establishes callback
 * @param numberOfShades used to size the number of shade states to track
 */
void HAShadeService::init(int numberOfShades) {
  ROS_INFO_STREAM(
      "HAShadeService initializing; " << numberOfShades << " shades to initialize.");
  // Store the number of shades in the first element of the shade state vector
  shadeState.push_back(numberOfShades);
  for (int d = 1; d <= numberOfShades; d++) {
    // Initialize all of the shade states to raised
    shadeState.push_back(homebot::HAShadeResponse::RAISED);
    ROS_INFO_STREAM("Shade " << d << " initialized to state " << shadeState[d]);
  }
  // Set the service server object using the node handle's advertiseService method,
  // the service name, and the callback method from this object
  ss = nh.advertiseService("ha_shade", &HAShadeService::callback, this);
}
