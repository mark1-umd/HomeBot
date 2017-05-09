/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file Repertoire.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 9, 2017 - Creation
 *
 * @brief A HomeBot repertoire is a collection of behaviors for a HomeBot service robot
 *
 * HomeBot represents a behavior as collections of operations that, when executed together,
 * produce activity that is described as the service robot carrying out the behavior.  Since
 * it is useful for a service robot to possibly have more than one behavior, a collection of
 * behaviors must be maintained for each service robot in a HomeBot system.  That collection
 * is termed a repertoire, and is maintained by this class.  The repertoire protects its
 * behaviors, only providing a copy of the behaviors when requested, not a reference or pointer
 * to the Repertoire's stored behavior.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_REPERTOIRE_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_REPERTOIRE_HPP_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "OperationParameters.hpp"
#include "homebot/BotBehavior.hpp"

/** @brief A collection of HomeBot service robot behaviors
 */

class Repertoire {
 public:
  Repertoire(const std::string& pBotType);
  virtual ~Repertoire();
  BotBehavior getBehavior(const std::string& pName);
  bool load(const std::string& pFilename, const OperationParameters& pOpParams);
 private:
  std::string botType;
  OperationParameters operationParameters;
  std::vector<BotBehavior> behaviors;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_REPERTOIRE_HPP_ */
