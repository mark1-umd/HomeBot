/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotAffectHAShade.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief Operation that commands Home Automation system to lower and raise shades
 *
 * In a HomeBot system, the Home Automation system is responsible for lowering/raising shades.
 * This operation provides a way for a HomeBot service robot to lower/raise shades through
 * the Home Automation system as part of a HomeBot behavior.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_BOTAFFECTHASHADEOPR_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_BOTAFFECTHASHADEOPR_HPP_

#include <string>
#include "ros/ros.h"
#include "homebot/HAShade.h"
#include "homebot/BotOperation.hpp"
#include "homebot/OperationParameters.hpp"
#include "homebot/BotOprClients.hpp"

/** @brief Provides a Home Automation "shade" operation (lower/raise); derived from BotOperation
 */

class BotAffectHAShadeOpr : public BotOperation {
 public:
  BotAffectHAShadeOpr();
  BotAffectHAShadeOpr(const std::string pCode, const int pShadeNumber,
                      const int pAction);
  virtual ~BotAffectHAShadeOpr();
  homebot::HAShade::Request details();
  bool isExecutable(const OperationParameters& opParams);
  bool execute(BotOprClients& clients);
 private:
  homebot::HAShade::Request request;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_BOTAFFECTHASHADEOPR_HPP_ */
