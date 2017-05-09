/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file Repertoire.cpp
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
#include "homebot/Repertoire.hpp"

Repertoire::Repertoire(const std::string& pBotType)
    : botType(pBotType),
      operationParameters() {
}

Repertoire::~Repertoire() {
}

/**
 * @brief Determine whether a particular behavior is in a repertoire of behaviors
 * @param [in] pName a string with a behavior name
 * @param [out] pBehavior the BotBehavior identified by the behavior name pName
 * @return
 */
BotBehavior Repertoire::getBehavior(const std::string& pName) {
  ROS_ERROR_STREAM(
      "HomeBot-Repertoire(getBehavior): Behavior '" << pName << "' requested from repertoire for '" << botType << "'");
  for (std::vector<BotBehavior>::size_type i = 0; i < behaviors.size(); i++) {
    if (behaviors[i].getName() == pName) {
      return behaviors[i];
    }
  }
  BotBehavior nullBehavior("", operationParameters);
  return nullBehavior;
}

bool Repertoire::load(const std::string& pFilename,
                      const OperationParameters& pOpParams) {
  // Update our memory of the operation parameters
  operationParameters = pOpParams;
  ROS_ERROR_STREAM(
      "HomeBot-Repertoire(load): Loading repertoire for BotType '" << botType << "' from file '" << pFilename << "'");

  std::ifstream rptFile(pFilename.c_str());
  if (!rptFile.is_open()) {
    ROS_ERROR_STREAM(
        "HomeBot-Repertoire(load): Could not open repertoire file '" << pFilename << "'");
    return false;
  }

  // Get the BotType from the first line of the repertoire file to verify that the
  // behaviors are appropriate for the Bot that will have this repertoire
  std::string rptBotType;
  if (!getline(rptFile, rptBotType)) {
    ROS_ERROR_STREAM(
        "HomeBot-Repertoire(load): Could not read botType from repertoire file '" << pFilename<< "'");
    rptFile.close();
    return false;
  }
  if (botType != rptBotType) {
    ROS_ERROR_STREAM(
        "HomeBot-Repertoire(load): This repertoire's BotType '" << botType << "' does not match the file BotType '" << rptBotType << "' in file '" << pFilename << "'");
    rptFile.close();
    return false;
  }

  // Read the lines of behaviors from the repertoire file, building this repertoire
  int behaviorCount(0), operationCount(0);
  std::string textRepertoire;
  while (getline(rptFile, textRepertoire)) {
    if (textRepertoire.length() == 0) {
      // Tolerate blank lines in repertoire file
      ROS_WARN_STREAM(
          "HomeBot-Repertoire(load): skipping blank line in file '" << pFilename << "'");
    }
    std::stringstream ssRepertoire;
    ssRepertoire.str(textRepertoire);
    std::string behaviorName;
    ssRepertoire >> behaviorName;
    std::string textPhasedBehavior;
    getline(ssRepertoire, textPhasedBehavior);
    if (textPhasedBehavior.length() == 0) {
      // We don't like behaviors on a line without an actual operation, but we will tolerate them
      ROS_ERROR_STREAM(
          "HomeBot-Repertoire(load): Skipping blank operation for behavior << '" << behaviorName << "'");
      break;
    }

    // See if this behavior exists already
    std::vector<BotBehavior>::size_type index;
    for (index = 0; index < behaviors.size(); index++) {
      if (behaviors[index].getName() == behaviorName) {
        break;
      }
    }
    // If it doesn't, add it as a new behavior
    if (index == behaviors.size()) {
      ROS_ERROR_STREAM(
          "HomeBot-Repertoire(load): Adding new behavior '" << behaviorName << "'");
      BotBehavior newBehavior(behaviorName, operationParameters);
      behaviors.push_back(newBehavior);
      index = behaviors.size() - 1;
      behaviorCount++;
    }
    // Then stick the rest of the line into this behavior as a phased operation
    if (!behaviors[index].insert(textPhasedBehavior)) {
      ROS_ERROR_STREAM(
          "HomeBot-Repertoire(load): Error inserting operation '" << textPhasedBehavior << "' into behavior '" << behaviorName << "', ending load");
      rptFile.close();
      return false;
    }
    operationCount++;
    // End of while(getLine)
  }
  ROS_ERROR_STREAM(
      "HomeBot-Repertoire(load): Loaded repertoire for '" << botType << "' from file '" << pFilename << "'; " << operationCount << " operations in " << behaviorCount << " behaviors");
  rptFile.close();
  return true;
}
