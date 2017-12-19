cmake_minimum_required(VERSION 2.8.3)

project(skiros2_msgs)

find_package(catkin REQUIRED COMPONENTS
genmsg
message_generation
actionlib
std_msgs
actionlib_msgs
trajectory_msgs
sensor_msgs
control_msgs
)

add_message_files(
  FILES
  Param.msg
  Property.msg
  Relation.msg
  Statement.msg
  Condition.msg
  SkillProgress.msg
  ResourceDescription.msg
  WmElement.msg
  WmMonitor.msg
  )

add_service_files(
  FILES
  ResourceGetDescriptions.srv
  WoQuery.srv
  WoModify.srv
  WmGet.srv
  WoLoadAndSave.srv
  WmSetRelation.srv
  WmQueryRelations.srv
  WmModify.srv
  SkillCommand.srv
  )

add_action_files(
  FILES
  AssignTask.action
  )

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime genmsg actionlib std_msgs actionlib_msgs trajectory_msgs sensor_msgs control_msgs
)
