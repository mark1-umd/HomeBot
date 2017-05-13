/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotMoveBaseOpr.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 4, 2017 - Creation
 *
 * @brief Operation that commands a HomeBot to navigate to a specified location
 *
 * In a HomeBot system, the HomeBot navigation stack is responsible for moving the Bot Base.
 * This operation provides a way for a HomeBot service robot to move to a specified location
 * as part of a HomeBot behavior.
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

#include "homebot/BotMoveBaseOpr.hpp"

/**
 * @brief Constructor with no arguments for BotMoveBaseOpr; creates a useless object that won't execute
 */
BotMoveBaseOpr::BotMoveBaseOpr() {
}

/**
 * @brief Constructor for BotMoveBaseOpr; builds operation that may execute
 * @param pCode std::string indicating operation code; must be BotMoveBase for this to execute
 * @param pSceneNumber integer for scene number to act on; must be within operational parameters to execute
 * @param pAction integer for action to take; must be valid action for this to execute
 */

/**
 * @brief Constructor for BotMoveBaseOpr; builds operation that may execute
 * @param pCode std::string indicating operation code; must be BotMoveBase for this to execute
 * @param pFrame_id std::string for the move_base frame_id within which the pose is to be executed
 * @param pXPos double pose X position
 * @param pYPos double pose Y position
 * @param pZPos double pose Z position
 * @param pXOrient double X orientation (Quaternion)
 * @param pYOrient double Y orientation (Quaternion)
 * @param pZOrient double Z orientation (Quaternion)
 * @param pWOrient double W orientation (Quaternion)
 */
BotMoveBaseOpr::BotMoveBaseOpr(const std::string pCode,
                               const std::string pFrame_id, const double pXPos,
                               const double pYPos, const double pZPos,
                               const double pXOrient, const double pYOrient,
                               const double pZOrient, const double pWOrient)
    : BotOperation("", pCode) {
  goal.target_pose.header.frame_id = pFrame_id;
  goal.target_pose.pose.position.x = pXPos;
  goal.target_pose.pose.position.y = pYPos;
  goal.target_pose.pose.position.z = pZPos;
  goal.target_pose.pose.orientation.x = pXOrient;
  goal.target_pose.pose.orientation.y = pYOrient;
  goal.target_pose.pose.orientation.z = pZOrient;
  goal.target_pose.pose.orientation.w = pWOrient;
}

BotMoveBaseOpr::~BotMoveBaseOpr() {
}

/**
 * @brief Provides access to operation details for testing purposes
 * @return goal formatted according to the ROS action definition for MoveBase
 */
move_base_msgs::MoveBaseGoal BotMoveBaseOpr::details() {
  return goal;
}

/**
 * @brief Validates that an operation is within acceptable parameters
 * @param [in] opParams provides operational parameters for the system used to verify this operation can execute
 * @return bool value indicating whether this operation can execute within the current HomeBot system
 */
bool BotMoveBaseOpr::isExecutable(const OperationParameters& opParams) {
  ROS_DEBUG_STREAM("HomeBot-BotAffectHAShadeOpr(isExecutable): Entered");
  // The only validation right now is to ensure the right opcode
  if (code == "BotMoveBase") {
    ROS_DEBUG_STREAM(
        "HomeBot-BotAffectHAShadeOpr(isExecutable): returning TRUE");
    return true;
  } else {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHAShadeOpr(isExecutable): Validation failed code '" << code << "'" << " frame: '" << goal.target_pose.header.frame_id << "' pose: '" << static_cast<double>(goal.target_pose.pose.position.x) << " " << static_cast<double>(goal.target_pose.pose.position.y) << " " << static_cast<double>(goal.target_pose.pose.position.z) << " " << static_cast<double>(goal.target_pose.pose.orientation.x) << " " << static_cast<double>(goal.target_pose.pose.orientation.y) << " " << static_cast<double>(goal.target_pose.pose.orientation.z) << " " << static_cast<double>(goal.target_pose.pose.orientation.w) << "'");
    return false;
  }
}

/**
 * @brief Performs the behavior embedded in this operation (BotMoveBase moves a robot base via its navigation system
 * @param [in] clients object containing the action/service clients used by BotOperations to carry out their function
 * @return bool indicating whether the operation executed successfully
 */
bool BotMoveBaseOpr::execute(BotOprClients& clients) {
  // Check whether the action is still available
  if (!clients.acBotMoveBase.isServerConnected()) {
    ROS_WARN_STREAM(
        "HomeBot-BotMoveBaseOpr(execute): move_base action server not ready when trying to move to frame: '"
            << goal.target_pose.header.frame_id << "' pose: '"
            << static_cast<double>(goal.target_pose.pose.position.x) << " "
            << static_cast<double>(goal.target_pose.pose.position.y) << " "
            << static_cast<double>(goal.target_pose.pose.position.z) << " "
            << static_cast<double>(goal.target_pose.pose.orientation.x) << " "
            << static_cast<double>(goal.target_pose.pose.orientation.y) << " "
            << static_cast<double>(goal.target_pose.pose.orientation.z) << " "
            << static_cast<double>(goal.target_pose.pose.orientation.w) << "'");
    return false;
  }

  ROS_INFO_STREAM(
      "HomeBot-BotMoveBaseOpr(execute): sending goal to move_base; frame: '"
          << goal.target_pose.header.frame_id << "' pose: '"
          << static_cast<double>(goal.target_pose.pose.position.x) << " "
          << static_cast<double>(goal.target_pose.pose.position.y) << " "
          << static_cast<double>(goal.target_pose.pose.position.z) << " "
          << static_cast<double>(goal.target_pose.pose.orientation.x) << " "
          << static_cast<double>(goal.target_pose.pose.orientation.y) << " "
          << static_cast<double>(goal.target_pose.pose.orientation.z) << " "
          << static_cast<double>(goal.target_pose.pose.orientation.w) << "'");
  // Send goal to move_base action server, then wait for result
  goal.target_pose.header.stamp = ros::Time::now();
  clients.acBotMoveBase.sendGoal(goal);
  clients.acBotMoveBase.waitForResult();

  if (clients.acBotMoveBase.getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM(
        "HomeBot-BotMoveBaseOpr(execute): move_base reports goal reached");
    return true;
  } else {
    ROS_WARN_STREAM("HomeBot-BotMoveBaseOpr(execute): Failed to reach goal");
    return false;
  }
}
