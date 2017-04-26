# HomeBot

## Overview

The HomeBot family of home automation robots is a product line concept for ACME robots.  The HomeBot system integrates best-of-breed individual robotic capabilities with each other, home automation systems, and humans.  Rather than trying to make a single robot with a too-versatile set of capabilities, elevating its cost and complexity, the HomeBot system uses best-of-breed robotic systems working together and in conjunction with household systems managed by home automation to manage domestic environments from small to large.

The HomeBot system can include products such as:

- ButtleBot – automated butler services (answers the door, greets guests, provides tele-presence, takes packages, delivers objects from room to room, and more)
- TrashBot – takes the trash out and down to the road for pickup by city services
- LawnBot – keeps lawns looking fresh cut
- WatchBot – patrols estate perimeters day and night from the ground or from the air

The key to the HomeBot system is a ROS node that can interface one or more HomeBot robotic products with a home automation system, allowing them to interact bi-directionally; the robots can make requests of and send notifications to the home automation system, and the home automation system can make requests of and send notifications to the robotic systems.  These communications take place through the ROS Actionlib, a non-blocking service interface well-suited for this type of integration.

## License

BSD 3-Clause License

Copyright (c) 2017, Mark Jenkins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Technology

- Ubuntu Linux 14.04 as a VirtualBox guest O/S on a macOS 10.12 host O/S as the development platform
- git version control system with GitHub as a centralized repo host
- The Robot Operating System (ROS) version Indigo Igloo
- Gazebo 3D rigid body simulator integrated with ROS
- Turtlebot simulation stack for Indigo Igloo
- C++ language using the gcc compiler with C++11/14 syntax and extensions
- cmake build system
- googletest testing
- Travis Continual Integration
- Coveralls coverage monitoring (in development)

## Dependencies


## Status

## Prerequisites

This ROS package has been built and tested for the Indigo-Igloo release of ROS.
In order to build and use it, you will need to have ROS Indigo-Igloo installed on your system, along with the ROS dependencies identified in the package.xml manifest file (roscpp, rospy, std_msgs, message_generation, actionlib).  The instructions in this README.md file assume that you are familiar with ROS and the ROS catkin build system.

To add the Turtlebot simulation stack to your ROS Indigo Igloo environment under Ubuntu 14.04:

- $ sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-turtlebot-apps ros-indogo-turtlebot-rviz-launchers

## Import tutorials into your ROS catkin workspace

To import the project into your catkin workspace, clone or download it into the src subdirectory of your catkin workspace directory.  Once this package is part of your catkin workspace, it will build along with any other packages you have in that workspace using the "catkin_make" command executed at the top-level directory of your catkin workspace.

## Testing using rostest

Level 2 integration testing of ROS nodes uses the Google Test framework combined with the rostest tool to test ROS nodes.  For the purpose of demonstrating these testing capabilities, a ROS testing node called "xxxxxxxxxxx" is provided.  "xxxxxxxxxxx" tests xxxxxxxxxxx.  To invoke the rostest capability at the node level, a test launch script called "xxxxxxxxxx" is provided.  To invoke the test at the command line:

    - rostest HomeBot xxxxxxxxxxxx.test
    
The output from this command is placed into a specially formatted XML file.  For debugging purposes, the
output can be sent directly to the terminal using this form of the command:

    - rostest --text HomeBot xxxxxxxxxxxxx.test
    
The rostest capability is also baked into the package's CMakeLists.txt build configuration file.  To build the test node, invoke the catkin workspace build command with the target "tests":

    - catkin_make tests

To both build and run the tests as part of the catkin build system, use:

    - catkin_make run_tests

