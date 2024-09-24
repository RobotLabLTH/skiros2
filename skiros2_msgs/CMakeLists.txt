cmake_minimum_required(VERSION 3.5)

project(skiros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Param.msg
  msg/Property.msg
  msg/Relation.msg
  msg/Statement.msg
  msg/Condition.msg
  msg/SkillProgress.msg
  msg/TreeProgress.msg
  msg/ResourceDescription.msg
  msg/WmElement.msg
  msg/WmMonitor.msg
  srv/ResourceGetDescriptions.srv
  srv/WoQuery.srv
  srv/WoModify.srv
  srv/WmGet.srv
  srv/WmInitScene.srv
  srv/WoLoadAndSave.srv
  srv/WmSetRelation.srv
  srv/WmQueryRelations.srv
  srv/WmModify.srv
  srv/SkillCommand.srv
  action/AssignTask.action
  action/TestAction.action
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
