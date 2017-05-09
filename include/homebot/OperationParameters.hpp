/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file OperationParameters.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date May 7, 2017 - Creation
 *
 * @brief A simple read-only object to pass parameters used to make sure operations are within limits when created
 *
 * When the Node using Bot Behaviors is created, sizing parameters for the Home Automation system are provided
 * so that operations against the Home Automation system can be checked at load time for compatibility.  In order
 * to prevent having specify multiple individual parameters, those parameters are captured in this object.  For
 * ease of access, all attributes are public.  To protect them, they are created in a constructor as constants.
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
#ifndef HOMEBOT_INCLUDE_HOMEBOT_OPERATIONPARAMETERS_HPP_
#define HOMEBOT_INCLUDE_HOMEBOT_OPERATIONPARAMETERS_HPP_

/** @brief Holds system parameters used to ensure Operations are built to respect the limits of the system
 */

class OperationParameters {
 public:
  OperationParameters(int doors, int scenes, int shades);
  virtual ~OperationParameters();
  const int maxDoorNumber;
  const int maxSceneNumber;
  const int maxShadeNumber;
};

#endif /* HOMEBOT_INCLUDE_HOMEBOT_OPERATIONPARAMETERS_HPP_ */
