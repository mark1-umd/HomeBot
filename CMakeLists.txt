cmake_minimum_required(VERSION 2.8.3)
project(homebot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
  FILES
  HADoor.srv
  HAScene.srv
  HAShade.srv
  HADemo.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
add_action_files(
  FILES
  HAOpenDoor.action
  HAHvac.action
  HBBehavior.action
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES homebot
CATKIN_DEPENDS actionlib message_generation roscpp std_msgs move_base_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(homebot
#   src/${PROJECT_NAME}/homebot.cpp
# )

## Declare a cpp executable
# add_executable(homebot_node src/homebot_node.cpp)
add_executable(HARequests src/HARequests.cpp)
add_executable(HAexerciser src/HAexerciser.cpp)
add_executable(HARequestServer src/HARequestServer.cpp
                               src/HADoorService.cpp
                               src/HASceneService.cpp
                               src/HAShadeService.cpp)
add_executable(HAHvacActionServer src/HAHvacActionServer.cpp
                                  src/HAHvacAction.cpp)
add_executable(HomeBot_Node src/HomeBot_Node.cpp
                            src/OperationParameters.cpp
                            src/Repertoire.cpp
                            src/BotOprClients.cpp
                            src/BotActor.cpp
                            src/BotBehavior.cpp
                            src/BotOperation.cpp
                            src/BotMoveBaseOpr.cpp
                            src/BotAffectHADoorOpr.cpp
                            src/BotAffectHASceneOpr.cpp
                            src/BotAffectHAShadeOpr.cpp)
add_executable(FakeMoveBaseServer src/FakeMoveBaseServer.cpp
                                  src/FakeMoveBaseAction.cpp)
add_executable(FakeMoveBaseExerciser src/FakeMoveBaseExerciser.cpp)
add_executable(HAHomeBotClient_Node src/HAHomeBotClient_Node.cpp
                                    src/HAClients.cpp
                                    src/HADemoService.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(homebot_node homebot_generate_messages_cpp)
add_dependencies(HARequests homebot_generate_messages_cpp)
add_dependencies(HAexerciser homebot_generate_messages_cpp)
add_dependencies(HAHvacActionServer homebot_generate_messages_cpp)
add_dependencies(HARequestServer homebot_generate_messages_cpp)
add_dependencies(HomeBot_Node homebot_generate_messages_cpp)
add_dependencies(FakeMoveBaseServer homebot_generate_messages_cpp)
add_dependencies(HAHomeBotClient_Node homebot_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(homebot_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(HARequests ${catkin_LIBRARIES})
target_link_libraries(HAexerciser ${catkin_LIBRARIES})
target_link_libraries(HARequestServer ${catkin_LIBRARIES})
target_link_libraries(HAHvacActionServer ${catkin_LIBRARIES})
target_link_libraries(HomeBot_Node ${catkin_LIBRARIES})
target_link_libraries(FakeMoveBaseServer ${catkin_LIBRARIES})
target_link_libraries(FakeMoveBaseExerciser ${catkin_LIBRARIES})
target_link_libraries(HAHomeBotClient_Node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS homebot homebot_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_homebot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## Add rostest tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest_gtest(
  HARequestServer_test
  test/HARequestServer.test
  test/HARequestServer_test.cpp
  )
  add_rostest_gtest(
  HAHvacActionServer_test
  test/HAHvacActionServer.test
  test/HAHvacActionServer_test.cpp
  )
  #add_rostest_gtest(
  #BotBehavior_Component_Standalone_test
  #test/BotBehavior_Component_Standalone.test
  #test/BotBehavior_Component_Standalone_test.cpp
  #src/BotOperation.cpp
  #src/OperationParameters.cpp
  #src/BotMoveBaseOpr.cpp
  #src/BotAffectHADoorOpr.cpp
  #src/BotAffectHASceneOpr.cpp
  #src/BotAffectHAShadeOpr.cpp
  #src/BotOprClients.cpp
  #)
  add_rostest_gtest(
  BotBehavior_Component_test
  test/BotBehavior_Component.test
  test/BotBehavior_Component_test.cpp
  src/BotOperation.cpp
  src/OperationParameters.cpp
  src/BotMoveBaseOpr.cpp
  src/BotAffectHADoorOpr.cpp
  src/BotAffectHASceneOpr.cpp
  src/BotAffectHAShadeOpr.cpp
  src/BotOprClients.cpp
  src/BotBehavior.cpp
  )
  add_rostest_gtest(
  Repertoire_test
  test/Repertoire.test
  test/Repertoire_test.cpp
  src/Repertoire.cpp
  src/BotOperation.cpp
  src/OperationParameters.cpp
  src/BotMoveBaseOpr.cpp
  src/BotAffectHADoorOpr.cpp
  src/BotAffectHASceneOpr.cpp
  src/BotAffectHAShadeOpr.cpp
  src/BotOprClients.cpp
  src/BotBehavior.cpp
  )
  add_rostest_gtest(
  BotActor_test
  test/BotActor.test
  test/BotActor_test.cpp
  src/BotActor.cpp
  src/Repertoire.cpp
  src/BotOperation.cpp
  src/OperationParameters.cpp
  src/BotMoveBaseOpr.cpp
  src/BotAffectHADoorOpr.cpp
  src/BotAffectHASceneOpr.cpp
  src/BotAffectHAShadeOpr.cpp
  src/BotOprClients.cpp
  src/BotBehavior.cpp
  )
  
  target_link_libraries(
  HARequestServer_test
  ${catkin_LIBRARIES})
  
  target_link_libraries(
  HAHvacActionServer_test
  ${catkin_LIBRARIES})
  
 # target_link_libraries(
 # BotBehavior_Component_Standalone_test
 # ${catkin_LIBRARIES}
 # )
  target_link_libraries(
  BotBehavior_Component_test
  ${catkin_LIBRARIES})
  
  target_link_libraries(
  Repertoire_test
  ${catkin_LIBRARIES})
  
  target_link_libraries(
  BotActor_test
  ${catkin_LIBRARIES})
  
  add_dependencies(
  HARequestServer_test
  HARequestServer
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )
  
  add_dependencies(
  HAHvacActionServer_test
  HAHvacActionServer
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )
  
 # add_dependencies(
 # BotBehavior_Component_Standalone_test
 # ${PROJECT_NAME}_generate_messages.cpp
 # ${catkin_EXPORTED_TARGETS}
 # )
  
  add_dependencies(
  BotBehavior_Component_test
  ${PROJECT_NAME}_generate_messages.cpp
  ${catkin_EXPORTED_TARGETS}
  )
  
  add_dependencies(
  BotActor_test
  ${PROJECT_NAME}_generate_messages.cpp
  ${catkin_EXPORTED_TARGETS}
  )
  
endif()
