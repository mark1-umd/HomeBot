/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file BotOperation.cpp
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
#include "homebot/BotOperation.hpp"

// Includes for derived classes referenced in this base class implementation file
#include "homebot/BotMoveBaseOpr.hpp"
#include "homebot/BotAffectHADoorOpr.hpp"
#include "homebot/BotAffectHASceneOpr.hpp"
#include "homebot/BotAffectHAShadeOpr.hpp"


BotOperation::BotOperation()
    : code("") {
}

BotOperation::BotOperation(const std::string pCode)
    : code(pCode) {
}

BotOperation::~BotOperation() {
}

/**
 * @brief See what code is in an operation - primarily for testing purposes
 * @return std::string code from this object
 */
std::string BotOperation::getCode() {
  return code;
}

/**
 * @brief Determine as closely as possible whether an operation is valid (can be executed)
 * @param [in] OperationParameters opParams that assist with validation
 * @return
 */
bool BotOperation::isValid(const OperationParameters& opParams) {
  ROS_ERROR_STREAM(
      "HomeBot-BotOperation(isValid): Base class virtual method called; possible error in system");
  // This method is only executed from the base object, which means it is not a valid operation
  return false;
}

/**
 * @brief BotOperation objects are self-executing themselves; but a base class object should not be executed
 * @param [in] BotOprClients clients to use for executing the operation
 * @return bool indication of whether the execution was successful (true) or not (false)
 */
bool BotOperation::execute(BotOprClients& clients) {
  // Executing a base object can't do anything because it does not have any operation details
  ROS_ERROR_STREAM(
      "HomeBot-BotOperation(execute): Base class virtual method called; possible error in system");
  return false;
}

/**
 * @brief Base class method to construct a derived class object from components passed in as a stringstream
 * @param [in] stringstream operationComponents that can be used to construct an operation
 * @param [in] OperationParameters opParams that are used to make sure an operation is within limits
 * @return
 */
boost::shared_ptr<BotOperation> BotOperation::makeOpr(
    std::stringstream& operationComponents,
                                    const OperationParameters& opParams) {
  // Get the operation code from the operation components parameter and construct the
  // right derived class of the operation based on the operation code and return it or
  // return a null operation of the base class that will fail validation when checked
  std::string oprCode;
  operationComponents >> oprCode;
  ROS_ERROR_STREAM(
      "HomeBot-BotOperation(makeOpr): Making operation with code '" << oprCode << ";");

  if (oprCode == "BotMoveBase") {
    // For stringstream input, use directly-defined numeric/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    std::string frame_id;
    double pX, pY, pZ, oX, oY, oZ, oW;
    operationComponents >> frame_id >> pX >> pY >> pZ >> oX >> oZ >> oW;
    boost::shared_ptr<BotMoveBaseOpr> newOpr(
        new BotMoveBaseOpr(oprCode, frame_id, pX, pY, pZ, oX, oY, oZ, oW));
    if (!newOpr->isValid(opParams)) {
      ROS_ERROR_STREAM(
          "HomeBot-BotOperation(makeOpr): Invalid BotMoveBase operation with pose: " << pX << " " << pY << " " << pZ << " " << oX << " " << oY << " " << oZ <<" " << oW;);
      boost::shared_ptr<BotOperation> nullOpr(new BotOperation);
      return nullOpr;
    }
    return newOpr;

  } else if (oprCode == "HADoor") {
    // For stringstream input, use clearly defined integer/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    int doorNumber, action;
    operationComponents >> doorNumber >> action;
    boost::shared_ptr<BotAffectHADoorOpr> newOpr(
        new BotAffectHADoorOpr(oprCode, doorNumber, action));
    if (!newOpr->isValid(opParams)) {
      ROS_ERROR_STREAM(
          "HomeBot-BotOperation(makeOpr): Invalid HADoor operation with door number '" << doorNumber << "', action '" << action << "'");
      boost::shared_ptr<BotOperation> nullOpr(new BotOperation);
      return nullOpr;
    }
    ROS_ERROR_STREAM(
        "HomeBot-BotOperation(makeOpr): Returning valid HADoor operation");
    return newOpr;

  } else if (oprCode == "HAScene") {
    // For stringstream input, use clearly defined integer/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    int sceneNumber, action;
    operationComponents >> sceneNumber >> action;
    boost::shared_ptr<BotAffectHASceneOpr> newOpr(
        new BotAffectHASceneOpr(oprCode, sceneNumber, action));
    if (!newOpr->isValid(opParams)) {
      ROS_ERROR_STREAM(
          "HomeBot-BotOperation(makeOpr): Invalid HAScene operation with scene number " << sceneNumber << ", action " << action);
      boost::shared_ptr<BotOperation> nullOpr(new BotOperation);
      return nullOpr;
    }
    return newOpr;

  } else if (oprCode == "HAShade") {
    // For stringstream input, use clearly defined integer/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    int shadeNumber, action;
    operationComponents >> shadeNumber >> action;
    boost::shared_ptr<BotAffectHAShadeOpr> newOpr(
        new BotAffectHAShadeOpr(oprCode, shadeNumber, action));
    if (!newOpr->isValid(opParams)) {
      ROS_ERROR_STREAM(
          "HomeBot-BotOperation(makeOpr): Invalid HAShade operation; shade number " << shadeNumber << ", action " << action);
      boost::shared_ptr<BotOperation> nullOpr(new BotOperation);
      return nullOpr;
    }
    return newOpr;

  }
  ROS_ERROR_STREAM(
      "HomeBot-BotOperation(makeOpr): Unrecognized operation '" << oprCode << "', can't decode operation");
  boost::shared_ptr<BotOperation> nullOpr(new BotOperation);
  return nullOpr;
}
