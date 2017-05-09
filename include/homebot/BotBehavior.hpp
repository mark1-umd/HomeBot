/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotBehavior.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 7, 2017 - Creation
 *
 * @brief A BotBehavior is a set of operations that are executed serially to create a behavior
 *
 * In the HomeBot system, HomeBot service robots can be tasked to perform behaviors.  Each behavior is a
 * series of individual operations that are executed serially to create the desired behavior.  Since
 * behaviors may be repeated, the operations are divided into three phases: preliminary, main, and post.
 * Within each phase, the set of operations are executed atomically.  A presumption of the arrangement is
 * that if a preliminary phase of operations for a behavior has been completed, the post phase of operations
 * must also be completed to end the behavior, even if the main phase is never executed.  The responsibility
 * for ensuring that this happens lies outside of the behavior itself, however.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_BOTBEHAVIOR_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_BOTBEHAVIOR_HPP_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <homebot/BotOprClients.hpp>
#include <homebot/BotOperation.hpp>
#include <homebot/OperationParameters.hpp>
#include <homebot/BotAffectHADoorOpr.hpp>
#include <homebot/BotAffectHASceneOpr.hpp>
#include <homebot/BotAffectHAShadeOpr.hpp>
#include <homebot/BotMoveBaseOpr.hpp>

/** @brief <brief description>
 */

class BotBehavior {
 public:
  BotBehavior(const std::string pName, const OperationParameters& pOpParams);
  virtual ~BotBehavior();
  std::string getName();
  bool insert(const std::string textPhasedOpr);
  bool performPrelim(BotOprClients& oprClients);
  bool performMain(BotOprClients& oprClients);
  bool performPost(BotOprClients& oprClients);
 private:
  std::string name;
  OperationParameters opParams;
  std::vector<boost::shared_ptr<BotOperation> > prelimOprs;
  std::vector<boost::shared_ptr<BotOperation> > mainOprs;
  std::vector<boost::shared_ptr<BotOperation> > postOprs;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_BOTBEHAVIOR_HPP_ */
