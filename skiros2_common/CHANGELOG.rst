^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package skiros2_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-06-09)
------------------
* hasRelation also checking without replacing -1
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Fix to `#24 <https://github.com/RVMI/skiros2/issues/24>`_
* Merge remote-tracking branch 'rvmi/develop' into develop
* Merge branch 'develop' into develop-rss
  # Conflicts:
  #	skiros2/scripts/install_fd_task_planner.sh
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Contributors: Matthias Mayr, francesco

1.0.4 (2020-03-04)
------------------
* Merge pull request `#23 <https://github.com/RVMI/skiros2/issues/23>`_ from RVMI/py3-fixes
  Py3 fixes
* Change name of method iteritems in param handler
* Merge pull request `#21 <https://github.com/RVMI/skiros2/issues/21>`_ from RVMI/common/python3-fixes
  Python3 compatibility fixes in skiros2_common
* Python3 compatibility fixes in skiros2_common
* Fix on GUI shortcuts and Element hasRelation function.
* Merge pull request `#20 <https://github.com/RVMI/skiros2/issues/20>`_ from RVMI/emk/fixes
  Emk/fixes
* Remove deprecated set import
* Fix bugs in hasProperty and setProperty
* Converted all enums from flufl.enum to standard enum. Removed dependency.
* Added assertions to guard skill execution return value
* Moved wmi to base skill class. Added function to determine if an element id refers to a template or an instantiated object.
* Removed obsolete make_instance option in getParamValue. Fix in property setValue.
* Added more documentation. Removed obsolete files. Moved generated files (e.g. logs and plans) into ~/.skiros folder. Simplified planner installation, that now doesn't modify system path.
* Cherrypick from emk/devel:
  Fix bug in setValues
  Fix bug in remap
* Bug fix in Element setProperty function
* Supporting input of lists in properties from the GUI
* Removed obsolete attribute _is_list from Property class.
* Finish improved-debugging-tools
* 1.0.3
* Contributors: Francesco Rovida, RvmiLab, emmanuelkring, Francesco Rovida

1.0.2 (2019-08-26)
------------------
* Merge branch 'master' into develop
* Finish gui-improved-visualization
* Printing SLOW tag to primitives running with a period above 40msec.
* Added requirements, updated readme and catching ImportError when loading skill plugins.
* Changed way to evaluate abstract conditions (now based on class).
* Updated changelog
* Revert "Bug fix in parameters copying"
  This reverts commit d2b7d2f99c57ab8a3f0da529758aa9b264c3c103.
* Contributors: Francesco, RvmiLab

1.0.1 (2019-05-22)
------------------
* Fixed bug in skill preemption
* Removed debug msg
* Fix in hold conditions (now checked before ticking childrens)
* Better output of conditions and reverted 2 changes causing problems.
* Bug-fix in gui with bool parameters
* Contributors: RvmiLab

1.0.0 (2019-04-30)
------------------
* onEnd is now called when a skill is preempted, not only when it ends by itself
* Planner uses hold conditions
* Removed unnecessary command
* Merge pull request `#11 <https://github.com/RVMI/skiros2/issues/11>`_ from matthiashh/matthias-devel-pycodestyle
  Matthias devel pycodestyle
* Merge branch 'develop' into matthias-devel-pycodestyle
* Added hold conditions to continuously evalutate a condition holds while ticking a skill
* Removed old BSD tags
* Merged 'develop' into 'pycodestyle'.
* Finish plan-in-the-tree
* Applied pycodestyle (pep8) with "--aggresive"
  This should fix most violations of pep8.
  Excemptions were made for manually structured code through tabs.
* Bug-fix in world_element append function and fixed doc tests.
* Extended Element hasProperty function and fixed skill manager output on preemption and execution failures.
* Visualizing blackboard data when a BT crashes. Limited property output print length, to avoid overloading the screen.
* Bug-fix in element removal
* Wm updates cached elements instead of deleting, for better performances
* Bug-fix in sequential processor and in Element addRelation
* Updated version and changelog file
* Updated setup.py for automatic installation of dependencies. Updated readme
* Contributors: Francesco Rovida, Matthias Mayr, RvmiLab, Francesco Rovida

0.1.0 (2018-09-27)
------------------
* Bug-fixes in task planning action and support for multiple skill libraries
* Finish wm-restructure
* added hasRelation method to Element class. Fixed world model get_relation to work with subproperties. Other minor cleaning.
* Override __call_\_ operator in skills for easier scripting. Automatically updating elements when adding to wm. Passing parameters to skills before exapanding.
* Added DictDiff utility. Fix in world element. Extended addChild function for skills
* Bug-fix and now removing an element twice doesn-t generate an error any more
* Changed world model interfaces methods to snake case
* Completed and tested implementation of wm contexts
* Started restructuring
* Removed obsolete package skiros_resource. Added Property msg and changed WmElement And WmMonitor msgs. Changed serialization accordingly.
* Cleaner implementation of skill's description modification
* Couple of fixes to specification of parameters
* Improvments on instanciation of skills.
* Added setRelation function
* Bug fix in remapping and several bug-fix on the GUI.
* Fixed bug when creating chained remappings of parameters.
* Simplified and cleaned memorization and print of skills' progresses. Now also printing the skill's parent name for clearer reading.
* Added time from start to skill progress.
* Bug-fix in GUI skill's parameters
* Improved debug output on GUI.
* Revert some changes.
* Further improvements to BT execution rate: decreased queries to wm and added interface to update only element properties. Bug fix on skills reset.
* Improvements to increase the BT ticking speed.
* Optimized GUI
* Optimized PDDL problem generation. Task planner returns success if replan is empty. Adding relation hasTemplate to all elements with a template. Added utility functions to world element and pddl interface.
* Changed check of relation condition
* Changes to discrete reasoners for better integration.
* Added guard in WM against relations with keys not in ontology. Improvement to pddl domain generation. Bug fix in parameters merging. Removed some debug msgs.
* Checking abstract relations when parameterizing skills in BT
* All unicode in params converted to str type. Fixed issue in task manager that sometime was not mapping the right value to the parameter key.
* Implemented replanning. Removed printouts on world model. Other minor improvements.
* Minor fixes
* Added unset function to params and removing parameter from map while merging if param is not specified.
* Reverted change to launch files. Changed evaluation of relation conditions. Bug fix in task manager interface.
* Changed Task manager control from service to action. Added verbosity option. Added support for universal quantifier.
* Changed condition name. Added startError function in primitives definition.
* Changed parameter types (`#10 <https://github.com/RVMI/skiros2/issues/10>`_)
  * Changed skill tab: added system parameters as mandatory and added empty field in optional parameters
  * Bug-fix in skill reset and adding element to world model.
  * Changed base parameters types. Removed World and Config became Required.
  * Added gui launch in world_model_server.launch
  * Fix avoiding gui crashing when a parameter is not specified
  * Removed System from ParamTypes
  * Setting name of the scene on the GUI according to init_scene parameter
* Removed System from ParamTypes
* Changed base parameters types. Removed World and Config became Required.
* Bug-fix in skill reset and adding element to world model.
* Bug-fixes for turtlesim launch and tf publishing when loading a scene (`#8 <https://github.com/RVMI/skiros2/issues/8>`_)
* Merge branch 'master' into master
* Bug-fix in stop of discrete reasoners
* Fixed load scene (`#4 <https://github.com/RVMI/skiros2/issues/4>`_)
  * Fix in reasoners stop function
* Fix in reasoners stop function
* Integration of AAU developments (`#3 <https://github.com/RVMI/skiros2/issues/3>`_)
  * Added support to run skiros under a ROS namespace. Updated readME.
  * Bug-fix when using a namespace. Implemented simple policy for wm to stay up until clients are correctly disconnected.
  * Added getIndividuals function to ontology interface
  * Started GUI for WM: Add objects
  * Bug-fix in getIndividuals function
  * Decreased ourput verbosity of skill manager
  * Handling spatial reasoner in properties of the world element
  * Access properties in world element
  * Changed isList function in property class.
  * Bug-fix for unicode/str serialization. Removed spaces in utils.py
  * Bug-fix on world model getIndividual. Removed spaces from world model.py
  * Development of GUI - WM tab
  * Started GUI for WM: Add objects
  * GUI development (wm add/remove/modify) in progress
  * Extended property condition with support for different operators. Added loop decorator. Minor change in world model server init.
  * Skill type is now set automatically to the class name
  * Fixed author name for world model modifcations. Fixed remove of objects
  * Changed all author_name with widget_id
  * Added interactive markers feedback to change to objects position
  * Bug-fix in param makeDefault and setValues functions
  * Bug-fix in world model load. On boot, skill manager remove old skill from the scene before adding new ones.
  * Skill in success state are no more bypassed by visitor (are executed again). Other minor changes on output
  * Checking file existence before loading scene.
  * Bug-fix
  * Added sequential operator and made it default.
  * Fixed bug in interactive markers
  * Added or condition. Change in param handle printState
  * Bug-fix in GUI set parameters. And changed interactive markers visualization
  * GUI update on wm changes
  * Fixed bugs with elements caching
  * Fixed bug in instanciate function. Fixed bug in the gui's add and remove object buttons.
  * Allow fast property update in GUI
  * Minor fixes
  * Bug-fix in skill manager naming
* Merge pull request `#1 <https://github.com/RVMI/skiros2/issues/1>`_ from ScalABLE40/master
  IPA changes integration
* Finish WP4_gui
  # Conflicts:
  #	skiros2_world_model/src/skiros2_world_model/ros/world_model_server.py
* GUI update on wm changes
* Added or condition. Change in param handle printState
* Bug-fix in world model load. On boot, skill manager remove old skill from the scene before adding new ones.
* Bug-fix in param makeDefault and setValues functions
* Skill type is now set automatically to the class name
* Extended property condition with support for different operators. Added loop decorator. Minor change in world model server init.
* 0.0.2
* Changelogs added
* GUI development (wm add/remove/modify) in progress
* Started GUI for WM: Add objects
* Bug-fix on world model getIndividual. Removed spaces from world model.py
* Bug-fix for unicode/str serialization. Removed spaces in utils.py
* Changed isList function in property class.
* Access properties in world element
* Handling spatial reasoner in properties of the world element
* Added support to run skiros under a ROS namespace. Updated readME.
* Merge pull request `#1 <https://github.com/RVMI/skiros2/issues/1>`_ from RVMI/master
  Added robot discovery mechanism and execution monitor
* Finish WP5_task_feedback (preliminary)
  # Conflicts:
  #	skiros2_common/src/skiros2_common/core/abstract_skill.py
* Added progress message and publisher
* Changed use of onStart (now returns True or False)
* new visitor to expand bt and retrieve skill sequence used to monitor task progress
* Finish WP5_robot_discovery (preliminary)
* Optimized some code;
  Added dummy sub and pub for robot discovery
* Bug-fix
* Skill's config params are no more communicated to the wm.
* First commit
* Contributors: Bjarne Grossmann, DavidWuthier, Francesco Rovida, Francesco Rovida, ipa-led
