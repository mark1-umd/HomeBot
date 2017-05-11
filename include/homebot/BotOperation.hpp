/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotOperation.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief This is a base class for all Bot Operations, the things a HomeBot can do using service/actions
 *
 * Bot Operation represent a kind of an instruction for a HomeBot, where the instruction includes not
 * only the OpCode, but the data for the operation as well.  A series of operations strung together forms
 * a behavior, and the set of behaviors for a given HomeBot type is that type's repertoire of behaviors.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_BOTOPERATION_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_BOTOPERATION_HPP_

#include <string>
#include <sstream>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "homebot/OperationParameters.hpp"
#include "homebot/BotOprClients.hpp"

// Forward declarations for derived classes referenced in this base class
class BotMOveBaseOpr;
class BotAffectHADoorOpr;
class BotAffectHASceneOpr;
class BotAffectHAShadeOpr;

/** @brief Base class for all operations; used to create new instances of derived classes from text-based source
 */

class BotOperation {
 public:
  BotOperation();
  BotOperation(const std::string pRawText);
  BotOperation(const std::string pRawText, const std::string pCode);
  virtual ~BotOperation();
  std::string getCode();
  std::string getRawText();
  virtual bool isExecutable(const OperationParameters& opParams);
  virtual bool execute(BotOprClients& clients);
  boost::shared_ptr<BotOperation> transform(
      const OperationParameters& opParams);
 protected:
  std::string code;
  std::string rawText;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_BOTOPERATION_HPP_ */
