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

BotMoveBaseOpr::BotMoveBaseOpr() {
  // TODO(Mark Jenkins): Auto-generated constructor stub
}

BotMoveBaseOpr::BotMoveBaseOpr(const std::string pCode,
                               const std::string pFrame_id, const int pXPos,
                               const int pYPos, const int pZPos,
                               const int pXOrient, const int pYOrient,
                               const int pZOrient, const int pWOrient)
    : BotOperation(pCode) {
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
  // TODO(Mark Jenkins): Auto-generated destructor stub

}

move_base_msgs::MoveBaseGoal BotMoveBaseOpr::details() {
  return goal;
}

bool BotMoveBaseOpr::isValid(const OperationParameters& opParams) {
  // The only validation right now is to ensure the right opcode
  if (code == "BotMoveBase")
    return true;
  else
  {
    ROS_WARN_STREAM(
        "HomeBot-BotAffectHAShadeOpr(isValid): Validation failed code '" << code << "'" << " frame: '" << goal.target_pose.header.frame_id << "' pose: '" << static_cast<double>(goal.target_pose.pose.position.x) << " " << static_cast<double>(goal.target_pose.pose.position.y) << " " << static_cast<double>(goal.target_pose.pose.position.z) << " " << static_cast<double>(goal.target_pose.pose.orientation.x) << " " << static_cast<double>(goal.target_pose.pose.orientation.y) << " " << static_cast<double>(goal.target_pose.pose.orientation.z) << " " << static_cast<double>(goal.target_pose.pose.orientation.w) << "'");
    return false;
  }
}
