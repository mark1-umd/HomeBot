/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file Repertoire.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief <brief description>
 *
 * <details>
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
#include "Repertoire.hpp"

Repertoire::Repertoire() {
  // TODO(Mark Jenkins): Auto-generated constructor stub

}

Repertoire::~Repertoire() {
  // TODO(Mark Jenkins): Auto-generated destructor stub

}
bool decodeOpr(std::stringstream &oprDetails, HBSysOpr opr) {
  std::string oprCode;
  oprDetails >> oprCode;
  if (oprCode == "BotMoveBase") {
    move_base_msgs::MoveBaseGoal details;
    oprDetails >> details.target_pose.pose.position.x
        >> details.target_pose.pose.position.y
        >> details.target_pose.pose.orientation.w;
    BotMoveBaseOpr newOpr(oprCode, details.target_pose.pose.position.x,
                          details.target_pose.pose.position.y,
                          details.target_pose.pose.orientation.w);
    if (!newOpr.isValid()) {
      ROS_ERROR_STREAM(
          "Invalid BotMoveBase operation " << details.target_pose.pose.position.x << " " << details.target_pose.pose.position.y << " " << details.target_pose.pose.orientation.w;);
      return false;
    }
  } else if (oprCode == "HADoor") {
    homebot::HADoorRequest details;
    oprDetails >> details.doorNumber >> details.action;
    HADoorAffectOpr newOpr(oprCode, details.doorNumber, details.action);
    if (!newOpr.isValid(maxDoorNumber)) {
      ROS_ERROR_STREAM(
          "Invalid HADoor operation; door number " << details.doorNumber << ", action " << details.action << " in repertoire file " << pFilename);
      return false;
    }
  } else if (oprCode == "HAScene") {
    homebot::HASceneRequest details;
    oprDetails >> details.sceneNumber >> details.action;
    HASceneAffectOpr newOpr(oprCode, details.sceneNumber, details.action);
    if (!newOpr.isValid(maxSceneNumber)) {
      ROS_ERROR_STREAM(
          "Invalid HAScene operation; scene number " << details.sceneNumber << ", action " << details.action << " in repertoire file " << pFilename);
      return false;
    }
  } else if (oprCode == "HAShade") {
    homebot::HAShadeRequest details;
    oprDetails >> details.shadeNumber >> details.action;
    HAShadeAffectOpr newOpr(oprCode, details.shadeNumber, details.action);
    if (!newOpr.isValid(maxShadeNumber)) {
      ROS_ERROR_STREAM(
          "Invalid HAShade operation; shade number " << details.shadeNumber << ", action " << details.action << " in repertoire file " << pFilename);
      return false;
    }
  }
}

bool Repertoire::load(std::string pBotType, std::string pFilename,
                      int maxDoorNumber, int maxSceneNumber,
                      int maxShadeNumber) {
  std::ifstream rptFile(pFilename);
  if (!rptFile) {
    ROS_FATAL_STREAM("Could not open repertoire file " << pFilename);
    return false;
  };
  std::string rptBotType;

  rptFile >> rptBotType;
  if (rptBotType != pBotType) {
    ROS_FATAL_STREAM(
        "Repertoire file is for " << rptBotType << ", expected repertoire for " << pBotType);
    return false;
  }

  botType = rptBotType;

  // Read the repertoire file, decode the instructions on each line, and add to the appropriate list
  std::string line;
  while (rptFile) {
    std::getline(rptFile, line);
    if (line.length() != 0) {
      std::stringstream oprLine;

      // Get the behavior name and stage
      std::string behaviorName, behaviorStage;
      oprLine >> behaviorName >> behaviorStage;

      // Decode the operation into an operation object
      HBSysOpr newOpr;
      if (!decodeOpr(line, newOpr)) {
        ROS_ERROR_STREAM("Could not decode operation");
        return false;
      }

    }

}

/**
 * @brief Determine whether a particular behavior is in a repertoire of behaviors
 * @param [in] pName a string with a behavior name
 * @param [out] pBehavior the BotBehavior identified by the behavior name pName
 * @return
 */
bool Repertoire::findBehavior(const std::string pName, BotBehavior &pBehavior) {
  for (std::vector<BotBehavior>::size_type i = 0; i < behaviors.size(); i++) {
    if (behaviors[i].getName() == pName) {
      pBehavior = behaviors[i];
      return true;
    }
  }
  return false;
}
