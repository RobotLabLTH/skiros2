cmake_minimum_required(VERSION 2.8.3)
project(skiros2)
find_package(catkin REQUIRED)
catkin_package()

# Version 1.0 (increment this comment to trigger a CMake update)
catkin_add_env_hooks(skiros SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#TODO: Planner script? env-hook?

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY owl launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/install_fd_task_planner.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)
