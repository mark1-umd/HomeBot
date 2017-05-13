/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file OperationParameters.cpp
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
#include "homebot/OperationParameters.hpp"

/**
 * @brief Constructor for OperationParameters; creates an object that is all zeros and so can't be used in a working system
 */
OperationParameters::OperationParameters()
    : maxDoorNumber(0),
      maxSceneNumber(0),
      maxShadeNumber(0) {
}

/**
 * @brief Constructor for OperationParameters; creates a useful object (assuming that the parameters are good)
 * @param doors integer representing the number of doors in a Home Automation system; used to validate HADoor operations
 * @param scenes integer representing the number of scenes in a Home Automation system; used to validate HAScene operations
 * @param shades integer representing the number of shades in a Home Automation system; used to validate HAShade operations
 */
OperationParameters::OperationParameters(int doors, int scenes, int shades)
    : maxDoorNumber(doors),
      maxSceneNumber(scenes),
      maxShadeNumber(shades) {
}

OperationParameters::~OperationParameters() {
}

