/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotBehavior.cpp
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
#include "homebot/BotBehavior.hpp"

/**
 * @brief Constructor for BotBehavior; will create a useless BotBehavior because it won't have a name
 */
BotBehavior::~BotBehavior() {
}

/**
 * @brief Constructor for BotBehavior; constructs a functional BotBehavior using the parameters
 * @param [in] pName std::string name for this behavior
 * @param [in] pOpParams OperationalParameters for this behavior
 */
BotBehavior::BotBehavior(const std::string pName,
                         const OperationParameters& pOpParams)
    : name(pName),
      opParams(pOpParams) {
}

/**
 * @brief Provides access to the behavior's name
 * @return string representing the behavior's name
 */
std::string BotBehavior::getName() {
  return name;
}

/**
 * @brief Adds an executable (transformed) operation to a phase of a behavior (phases are prelim(inary), main, and post)
 * @param [in] textPhasedOpr containing an operation phase and an operation code and details
 * @return bool indicating whether the operation was inserted successfully
 */
bool BotBehavior::insert(const std::string textPhasedOpr) {
  if (textPhasedOpr.length() == 0) {
    ROS_WARN_STREAM(
        "HomeBot-BotBehavior(insert): Invoked with zero-length phase behavior component text");
    return false;
  }
  ROS_DEBUG_STREAM(
      "HomeBot-BotBehavior(insert): Invoked with behavior '" << name << "' and text '" << textPhasedOpr << "'");
  std::stringstream ssPhasedOpr;
  ssPhasedOpr.str(textPhasedOpr);
  std::string phase;
  ssPhasedOpr >> phase;
  std::string textOpr;
  getline(ssPhasedOpr, textOpr);

  // See if we can turn the text into an executable operation
  ROS_DEBUG_STREAM(
      "HomeBot-BotBehavior(insert): Creating behavior '" << name << "' phase '" << phase << "' text '" << textOpr << "'");
  BotOperation rawOpr(textOpr);
  boost::shared_ptr<BotOperation> exeOpr = rawOpr.transform(opParams);
  if (!exeOpr->isExecutable(opParams)) {
    ROS_WARN_STREAM(
        "HomeBot-BotBehavior(insert): Failed to create executable operation '" << exeOpr->getCode() << "' for behavior '" << name << "'");
    return false;
  }
  ROS_DEBUG_STREAM(
      "HomeBot-BotBehavior(insert): Created executable operation '" << exeOpr->getCode() << "' for behavior '" << name << "'");

  // See if we can insert the executable operation into a recognized phase of this behavior
  if (phase == "prelim") {
    ROS_DEBUG_STREAM(
        "HomeBot-BotBehavior(insert): Adding operation '" << exeOpr->getCode() << "' to behavior '" << name << "' preliminary phase");
    prelimOprs.push_back(exeOpr);
    return true;
  } else if (phase == "main") {
    ROS_DEBUG_STREAM(
        "HomeBot-BotBehavior(insert): Adding operation '" << exeOpr->getCode() << "' to behavior '" << name << "' main phase");
    mainOprs.push_back(exeOpr);
    return true;
  } else if (phase == "post") {
    ROS_DEBUG_STREAM(
        "HomeBot-BotBehavior(insert): Adding operation '" << exeOpr->getCode() << "' to behavior '" << name << "' post phase");
    postOprs.push_back(exeOpr);
    return true;
  } else {
    ROS_WARN_STREAM(
        "Homebot-BotBehavior(insert): Invoked with unrecognized phase '" << phase << "', with '" << textOpr << "'");
    return false;
  }
}

/**
 * @brief Execute the operations in the preliminary phase of this behavior
 * @param oprClients object containing the action/service clients used by BotOperations to carry out their function
 * @return bool indicating whether the phase completed successfully
 */
bool BotBehavior::performPrelim(BotOprClients& oprClients) {
  // Keep track of whether all behaviors execute without trouble
  bool allGood = true;

  // Execute all behaviors in order
  for (std::vector<boost::shared_ptr<BotOperation> >::size_type i = 0;
      i < prelimOprs.size(); i++) {
    ROS_DEBUG_STREAM(
        "BotBehavior(performPrelim): Executing '" << prelimOprs[i]->getCode() << "'operation in this phase of behavior '" << name << "'");
    if (!prelimOprs[i]->execute(oprClients)) {
      ROS_WARN_STREAM(
          "HomeBot-BotBehavior(performPrelim): Failed to execute '" << prelimOprs[i]->getCode() << "' operation in this phase of behavior '" << name << "'");
      allGood = false;
    }
  }

  // Signal whether we were all good or not
  if (allGood) {
    ROS_DEBUG_STREAM(
        "HomeBot-BotBehavior(performPrelim): All operations in this phase of behavior '" << name << "' executed with no problems");
    return true;
  } else {
    ROS_WARN_STREAM(
        "HomeBot-BotBehavior(performPrelim): At least one failed operation in this phase of behavior '" << name << "'");
    return false;
  }
}

/**
 * @brief Execute the operations in the main phase of this behavior
 * @param oprClients object containing the action/service clients used by BotOperations to carry out their function
 * @return bool indicating whether the phase completed successfully
 */
bool BotBehavior::performMain(BotOprClients& oprClients) {
  // Keep track of whether all behaviors execute without trouble
  bool allGood = true;

  // Execute all behaviors in order
  for (std::vector<boost::shared_ptr<BotOperation> >::size_type i = 0;
      i < mainOprs.size(); i++) {
    ROS_DEBUG_STREAM(
        "BotBehavior(performMain): Executing '" << mainOprs[i]->getCode() << "'operation in this phase of behavior '" << name << "'");
    if (!mainOprs[i]->execute(oprClients)) {
      ROS_WARN_STREAM(
          "HomeBot-BotBehavior(performMain): Failed to execute '" << mainOprs[i]->getCode() << "' operation in this phase of behavior '" << name << "'");
      allGood = false;
    }
  }

  // Signal whether we were all good or not
  if (allGood) {
    ROS_DEBUG_STREAM(
        "HomeBot-BotBehavior(performMain): All operations in this phase of behavior '" << name << "' executed with no problems");
    return true;
  } else {
    ROS_WARN_STREAM(
        "HomeBot-BotBehavior(performMain): At least one failed operation in this phase of behavior '" << name << "'");
    return false;
  }
}

/**
 * @brief Execute the operations in the post phase of this behavior
 * @param oprClients object containing the action/service clients used by BotOperations to carry out their function
 * @return bool indicating whether the phase completed successfully
 */
bool BotBehavior::performPost(BotOprClients& oprClients) {
  // Keep track of whether all behaviors execute without trouble
  bool allGood = true;

  // Execute all behaviors in order
  for (std::vector<boost::shared_ptr<BotOperation> >::size_type i = 0;
      i < postOprs.size(); i++) {
    ROS_DEBUG_STREAM(
        "BotBehavior(performPost): Executing '" << postOprs[i]->getCode() << "'operation in this phase of behavior '" << name << "'");
    if (!postOprs[i]->execute(oprClients)) {
      ROS_WARN_STREAM(
          "HomeBot-BotBehavior(performPost): Failed to execute '" << postOprs[i]->getCode() << "' operation in this phase of behavior '" << name << "'");
      allGood = false;
    }
  }

  // Signal whether we were all good or not
  if (allGood) {
    ROS_DEBUG_STREAM(
        "HomeBot-BotBehavior(performPost): All operations in this phase of behavior '" << name << "' executed with no problems");
    return true;
  } else {
    ROS_WARN_STREAM(
        "HomeBot-BotBehavior(performPost): At least one failed operation in this phase of behavior '" << name << "'");
    return false;
  }
}

