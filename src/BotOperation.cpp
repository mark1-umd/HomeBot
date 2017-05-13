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

/**
 * Constructor for BotOperation; with no arguments it creates a BotOperation that is a null base object (not executable)
 */
BotOperation::BotOperation()
    : code(""),
      rawText("") {
}

/**
 * @brief Constructor for BotOperation; creates an instance of the base class that is possibly capable of being transformed into a derived class executable object
 * @param pRawText std::string containing a possible operation code and details that could be transformed into an executable operation
 */
BotOperation::BotOperation(const std::string pRawText)
    : rawText(pRawText) {
}

/**
 * Constructor for BotOperation; creates an instance of the base class that is possibly capable of being transformed into a derived class object that executable
 * @param pRawText std::string containing only the details of the operation specified in the pCode parameter
 * @param pCode std::string containing an operation code that may be recognized as a valid operation code during transformation
 */
BotOperation::BotOperation(const std::string pRawText, const std::string pCode)
    : code(pCode),
      rawText(pRawText) {
}

BotOperation::~BotOperation() {
}

/**
 * @brief Provides access to the code in an operation through the base class - primarily for testing purposes
 * @return std::string code from this object
 */
std::string BotOperation::getCode() {
  return code;
}

/**
 * @brief Determine as closely as possible whether an operation is valid (can be executed)
 * @param [in] OperationParameters opParams that assist with validation
 * @return bool value that indicates whether the operation can be executed
 */
bool BotOperation::isExecutable(const OperationParameters& opParams) {
  ROS_WARN_STREAM(
      "HomeBot-BotOperation(isExecutable): Base class virtual method called; possible error in system");
  // This method is being called in the base object, which means it is not an executable operation
  return false;
}

/**
 * @brief BotOperation objects are self-executing themselves; but a base class object should not be executed
 * @param [in] BotOprClients clients to use for executing the operation
 * @return bool indication of whether the execution was successful (true) or not (false)
 */
bool BotOperation::execute(BotOprClients& clients) {
  // Executing a base object can't do anything because it has not been transformed to executable form
  ROS_ERROR_STREAM(
      "HomeBot-BotOperation(execute): Base class virtual method called; probable error in system");
  return false;
}

/**
 * @brief Base class method to construct a derived class object from source text (inserted by constructor)
 * @param [in] opParams  provides operational parameters for the system used to verify this operation
 * @return boost::shared_ptr<BotOperation> for the derived class object (non-executable base object returned if transformation failed)
 */
boost::shared_ptr<BotOperation> BotOperation::transform(
    const OperationParameters& opParams) {
  // Transform the operation's raw text into an operation-specific ready-to-execute form

  // If we don't have a code yet, and there is no raw text, so no transformation is possible
  if ((code == "") && (rawText.length() == 0)) {
    ROS_WARN_STREAM(
        "HomeBot-BotOperation(transform): Attempt to transform an operation with no code and no raw text");
    return boost::shared_ptr<BotOperation>(new BotOperation(rawText, code));
  }

  // Either we have a code, or there is raw text that should start with a code, so we can begin
  std::stringstream operationComponents;
  operationComponents.str(rawText);

  // If we don't have a code, it should be the first thing in the raw text
  if (code == "") {
    operationComponents >> code;
  }
  ROS_DEBUG_STREAM(
      "HomeBot-BotOperation(transform): Making operation with code '" << code << "' and raw text '" << rawText << "'");

  if (code == "BotMoveBase") {
    // For stringstream input, use directly-defined numeric/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    std::string frame_id;
    double pX, pY, pZ, oX, oY, oZ, oW;
    operationComponents >> frame_id >> pX >> pY >> pZ >> oX >> oZ >> oW;
    // Use derived class constructor to embed details into the operation
    boost::shared_ptr<BotMoveBaseOpr> newOpr(
        new BotMoveBaseOpr(code, frame_id, pX, pY, pZ, oX, oY, oZ, oW));
    if (!newOpr->isExecutable(opParams)) {
      ROS_WARN_STREAM(
          "HomeBot-BotOperation(transform): Invalid BotMoveBase operation with pose: " << pX << " " << pY << " " << pZ << " " << oX << " " << oY << " " << oZ <<" " << oW;);
      // Return a non-executable base class object populated with what the current base class object contained
      return boost::shared_ptr<BotOperation>(new BotOperation(rawText, code));
    }
    ROS_DEBUG_STREAM(
        "HomeBot-BotOperation(transform): Returning valid BotMoveBase operation");
    return newOpr;

} else if (code == "HADoor") {
    // For stringstream input, use clearly defined integer/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    int doorNumber, action;
    operationComponents >> doorNumber >> action;
    // Use derived class constructor to embed details into the operation
    boost::shared_ptr<BotAffectHADoorOpr> newOpr(
      new BotAffectHADoorOpr(code, doorNumber, action));
    if (!newOpr->isExecutable(opParams)) {
      ROS_WARN_STREAM(
          "HomeBot-BotOperation(transform): Invalid HADoor operation with door number '" << doorNumber << "', action '" << action << "'");
      // Return a non-executable base class object populated with what the current base class object contained
      return boost::shared_ptr<BotOperation>(new BotOperation(rawText, code));
    }
    ROS_DEBUG_STREAM(
        "HomeBot-BotOperation(transform): Returning valid HADoor operation");
    return newOpr;

} else if (code == "HAScene") {
    // For stringstream input, use clearly defined integer/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    int sceneNumber, action;
    operationComponents >> sceneNumber >> action;
    // Use derived class constructor to embed details into the operation
    boost::shared_ptr<BotAffectHASceneOpr> newOpr(
      new BotAffectHASceneOpr(code, sceneNumber, action));
    if (!newOpr->isExecutable(opParams)) {
      ROS_WARN_STREAM(
          "HomeBot-BotOperation(transform): Invalid HAScene operation with scene number " << sceneNumber << ", action " << action);
      // Return a non-executable base class object populated with what the current base class object contained
      return boost::shared_ptr<BotOperation>(new BotOperation(rawText, code));
    }
    ROS_DEBUG_STREAM(
        "HomeBot-BotOperation(transform): Returning valid HAScene operation");
    return newOpr;

} else if (code == "HAShade") {
    // For stringstream input, use clearly defined integer/string variables, not the ROS message types - some
    // of these types (such as int8_t) interact poorly with streamstream, causing a character value to be read
    // instead of the actual desired integer value
    int shadeNumber, action;
    operationComponents >> shadeNumber >> action;
    // Use derived class constructor to embed details into the operation
    boost::shared_ptr<BotAffectHAShadeOpr> newOpr(
        new BotAffectHAShadeOpr(code, shadeNumber, action));
    if (!newOpr->isExecutable(opParams)) {
      ROS_WARN_STREAM(
          "HomeBot-BotOperation(transform): Invalid HAShade operation; shade number " << shadeNumber << ", action " << action);
      // Return a non-executable base class object populated with what the current base class object contained
      return boost::shared_ptr<BotOperation>(new BotOperation(rawText, code));
    }
    ROS_DEBUG_STREAM(
        "HomeBot-BotOperation(transform): Returning valid HAShade operation");
    return newOpr;

  }
  ROS_INFO_STREAM(
      "HomeBot-BotOperation(transform): Unrecognized operation '" << code << "', can't decode operation");
  // Return a non-executable base class object populated with what the current base class object contained
  return boost::shared_ptr<BotOperation>(new BotOperation(rawText, code));
}
