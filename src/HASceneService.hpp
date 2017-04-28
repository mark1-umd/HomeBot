/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file HASceneService.hpp
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
#ifndef HOMEBOT_SRC_HASCENESERVICE_HPP_
#define HOMEBOT_SRC_HASCENESERVICE_HPP_

#include <vector>
#include "ros/ros.h"
#include "homebot/HAScene.h"


/** @brief Provides HA Scene Service to a ROS node acting as a Home Automation Request Server
 */

class HASceneService {
 public:
  HASceneService();
  virtual ~HASceneService();
  bool callback(homebot::HAScene::Request& req,
                homebot::HAScene::Response& rsp);
  void init(int numberOfScenes);
 private:
  ros::NodeHandle nh;
  ros::ServiceServer ss;
  std::vector<int> sceneState;
};

#endif /* HOMEBOT_SRC_HASCENESERVICE_HPP_ */
