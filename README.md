# HomeBot

## Overview

The HomeBot family of home automation robots is a product line concept for ACME robots.  The HomeBot system integrates best-of-breed individual robotic capabilities with each other, home automation systems, and humans.  Rather than trying to make a single robot with a too-versatile set of capabilities, elevating its cost and complexity, the HomeBot system uses best-of-breed robotic systems working together and in conjunction with household systems managed by home automation to manage domestic environments from small to large.

The HomeBot system can include products such as:

- ButtleBot – automated butler services (answers the door, greets guests, provides tele-presence, takes packages, delivers objects from room to room, and more)
- TrashBot – takes the trash out and down to the road for pickup by city services
- LawnBot – keeps lawns looking fresh cut
- WatchBot – patrols estate perimeters day and night from the ground or (future product) from the air

The key to the HomeBot system is a ROS node that can interface one or more HomeBot robotic products with a home automation system, allowing them to interact bi-directionally; the robots can make requests of and send notifications to the home automation system, and the home automation system can make requests of and send notifications to the robotic systems.  These communications take place through the ROS Actionlib, a non-blocking service interface well-suited for this type of integration.

- SIP process metrics required for my final project development period (limited access): https://docs.google.com/spreadsheets/d/1AOjCXkzn5bVNjql5yxMNhNj-ozG26R38qnCTTYffQp0/edit?usp=sharing

- SIP process planning/review notes required for my final project development period (limited access): https://docs.google.com/document/d/1ytfDb1QFfRs0qY4guM0NiX4fi4OJUcFbAuXwF2xlH6Q/edit?usp=sharing

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
- [future] Gazebo 3D rigid body simulator integrated with ROS
- [future] Turtlebot simulation stack for Indigo Igloo
- C++ language using the gcc compiler with C++11/14 syntax and extensions
- cmake build system
- googletest testing
- [future] Travis Continual Integration
- [future] Coveralls coverage monitoring (in development)

## Dependencies
At this stage of development, HomeBot depends on standard ROS components, including:
- roscpp
- actionlib
- std_msgs
- geometry_msgs
- move_base_msgs
- message_generation

In the future, integration with the Turtlebot Gazebo simulation environment is envisioned as a way to visualize and demonstrate HomeBot service robot behaviors.

## Status

HomeBot has been developed to the point where it is a technology demonstration that shows what could be possible as an integration between Home Automation systems and service robots.  Integration with an actual Home Automation system, and with physical robots acting in the real world, is a future effort.

## Prerequisites

This ROS package has been built and tested for the Indigo-Igloo release of ROS.
In order to build and use it, you will need to have ROS Indigo-Igloo installed on your system, along with the ROS dependencies identified in the package.xml manifest file (roscpp, rospy, std_msgs, message_generation, actionlib).  The instructions in this README.md file assume that you are familiar with ROS and the ROS catkin build system.

To add the Turtlebot simulation stack to your ROS Indigo Igloo environment under Ubuntu 14.04:

- $ sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-turtlebot-apps ros-indogo-turtlebot-rviz-launchers

## Import HomeBot into your ROS catkin workspace

To import the project into your catkin workspace, clone or download it into the src subdirectory of your catkin workspace directory.  Once this package is part of your catkin workspace, it will build along with any other packages you have in that workspace using the "catkin_make" command executed at the top-level directory of your catkin workspace.

## Major Components

This demonstration system is heavily-based on ROS in its current form.  ROS communications (services and the action protocol, which are themselves based on ROS publish/subscribe messaging) form the heart of the system.  Custom services and actions are defined (see the srv and action sub-directories) for HomeBot use, in addition to using standard and commonly available message types (such as the move_base_msgs).  A new type of subdirectory and file is added to this ROS package: the repertoire subdirectory, which contains the .rpt files that express behaviors as a series of operations within the HomeBot system.

In addition to ROS, the major components of the system are described below.

### HARequestServer

The HARequestServer provides a way for ROS nodes to request services from a Home Automation system.  Integration capabilities currently demonstrated are include opening/closing doors,  turning lighting scenes on/off, and and lowering/raising window shades.

### HAHVAC Action Server

The HA HVAC Action server shows how a goal-oriented Home Automation capability (setting heat/cool modes with desired temperatures) can be integrated into the ROS system.  None of the currently demonstrated behaviors currently take advantage of this capability.

### HomeBot_Node

The HomeBot Node is the behavior action server for the HomeBot service robots.  When fully developed, an individual HomeBot_Node will be present in the HomeBot system for each HomeBot service robot.  The behavior action server loads a Repertoire of custom behaviors for each service robot, then exposes the behaviors through a ROS action server.  A Home Automation system, interfacing with the HomeBot system through a HABotBehaviorClient, can signal HomeBot service robots to perform behaviors either on schedule (such as taking trash out to the curb for pickup), or in response to sensors that the Home Automation system manages (such as a doorbell being pressed by a visitor causing a ButtleBot to answer the door, or a disturbance in the yard causing a WatchBot to perform a perimeter patrol.

The HomeBot_Node represents the major technology implementation in the demonstration package.  It creates a HomeBot operation instruction capability.  The operations are grouped together to form behaviors.  Behaviors are grouped together to form a repertoire for each service robot.  The behaviors are stored in a file, then loaded into working memory as a repertoire when the HomeBot_Node for a service robot is started up.  Once the service and action clients have been started on the HomeBot_Node, and the repertoire of behavior has been loaded, the HomeBot_Node exposes the interface to its repertoire of behaviors through a custom action protocol that expresses a behavior as a goal, with a requested number of repetitions of the behavior to be performed.  Once a requester has initiated a behavior in this manner, the HomeBot_Node executes the operation instructions in sequence to perform the behavior.

### HABotBehaviorClient

The HABotBehaviorClient integrates a Home Automation system into the HomeBot system by providing a pathway for behaviors requested by the Home Automation system to be activated (as a ROS action protocol goal) on a particular HomeBot service robot.  Once the behaviors are activated on a HomeBot_Node belonging to a particular service robot, the behaviors may act both on the robot (such as move_base goals that direct the robot to different locations), or back on the Home Automation system (allowing a behavior to perform tasks such as opening doors, turning lights on and off, or raising/lowering shades).  It is this last capability that reduces the need for a highly-capable robot mechanism capable of operating in the human world. The HomeBot service robots actuate things through the Home Automation system when necessary, instead of having local manipulators capable of interacting with the wide variety of human to touch interfaces.

### FakeMoveBaseServer

The FakeMoveBaseServer is a stop-gap measure created to work around a limitation of using Gazebo/Turtlebot on my development ROS VM.  Transform publishing did not work properly, so navigation of the simulated Turtlebot was not possible.  FakeMoveBaseServer provides a "move_base" action server that crudely represents what the real move_base action server provides with an actual navigation system.  It accepts full Pose goals, but only uses the (X,Y) component.  It initializes at location (0,0), then tracks/maintains state of its location as it is directed to different locations using move_base goals.  It simulates a physical robot moving at 1 meter/second (can be changed in the source code), providing position feedback at 10 Hz from when a goal is requested to when the goal is reached.

## Demonstrations

Two demonstrations are currently provided:

- HomeBot_System_ButtleBot.launch: starts up a HomeBot system using a ButtleBot repertoire for the HomeBot Node.
- HomeBot_System_WatchBot.launch: starts up a HomeBot system using a WatchBot repertoire for the HomeBot_Node.
## Testing using rostest

Level 2 integration testing of ROS nodes uses the Google Test framework combined with the rostest tool to run the ROS nodes individually or in groups.  This package uses the testing capability extensively, both for testing individual components and for testing combinations of components.

### Tests available

- HARequestServer.test: Verifies the function of the HARequestServer node using a ROS test node to drive the HARequestServer node
- HAHvacActionServer.test: Verifies the function of the HAHvacActionServer node using a ROS test node to drive the HAHvacActionServer node
- BotBehavior_Component_Standalone.test: Verifies the function of multiple components that together make up the BotBehavior capability when they are operated without the ROS nodes that provide interaction support
- BotBehavior_Component.test: Verifies the function of multiple components that together make up the BotBehavior capability when operated with other ROS nodes that interact with the components
- Repertoire.test: Verifies the function of the Repertoire component, which depends upon the BotBehavior components; does not use any other nodes. THIS TEST HAS HARD CODED FILEPATHS in the Repertoire_test.cpp source code because no ready method was apparent to pass arguments in through the rostest/gtest execution framework to the code inside of the test macros.
- BotActor.test: Verifies the function of the BotActor class (the behavior action server) in conjunction with many other components (BotBehavior, Repertoire); requires HARequestServer and FakeMoveBaseServer for testing. THIS TEST HAS HARD CODED FILEPATHS in the Repertoire_test.cpp source code because no ready method was apparent to pass arguments in through the rostest/gtest execution framework to the code inside of the test macros.

### General testing instructions:
The use of these test capabilities assumes a basic familiarity with the ROS sytem, including the rostest capability.  Once the homebot package has been integrated into a catkin workspace, the following commands can be used.

To invoke a test at the command line:

    - rostest homebot HARequestServer.test
    
The output from this command is placed into a specially formatted XML file.  For debugging purposes, the
output can be sent directly to the terminal using this form of the command:

    - rostest --text HomeBot xxxxxxxxxxxxx.test
    
The rostest capability is baked into the package's CMakeLists.txt build configuration file.  To build the test nodes, invoke the catkin workspace build command with the target "tests":

    - catkin_make tests

To both build the test nodes and run the tests as part of the catkin build system, use:

    - catkin_make run_tests

