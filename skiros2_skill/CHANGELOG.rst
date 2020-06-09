^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package skiros2_skill
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-06-09)
------------------
* Skill manager prints out available skills from packages in verbose mode.
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Merge remote-tracking branch 'rvmi/develop' into develop
* Merge branch 'develop' into develop-rss
  # Conflicts:
  #	skiros2/scripts/install_fd_task_planner.sh
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Contributors: Matthias Mayr, francesco

1.0.4 (2020-03-04)
------------------
* Merge pull request `#23 <https://github.com/RVMI/skiros2/issues/23>`_ from RVMI/py3-fixes
  Py3 fixes
* Change name of method iteritems in param handler
* Python3 fixes in skill_manager
* Make exception handling py3 compatible
* Python3 fixes in skiros2_skill
* Converted all enums from flufl.enum to standard enum. Removed dependency.
* Moved wmi to base skill class. Added function to determine if an element id refers to a template or an instantiated object.
* Added SerialStar processor and deprecated Sequential (still there as an alias). Comments to task manager interface
* Added wmi property to skill.
* Added more documentation. Removed obsolete files. Moved generated files (e.g. logs and plans) into ~/.skiros folder. Simplified planner installation, that now doesn't modify system path.
* Finish improved-debugging-tools
* Fixed bug leaving composite skills in running state and resetting processor.
* Removed obsolete function
* 1.0.3
* Updated changelog
* Calling onEnd function also when skill crashes. Removed filter to skills progress output in skill manager.
* Sending processor in skill progress
* Contributors: Emmanuel Kring, Francesco Rovida

1.0.3 (2019-09-16)
------------------
* Calling onEnd function also when skill crashes. Removed filter to skills progress output in skill manager.
* Sending processor in skill progress
* Contributors: Francesco Rovida

1.0.2 (2019-08-26)
------------------
* Merge branch 'master' into develop
* Finish gui-improved-visualization
* Printing SLOW tag to primitives running with a period above 40msec.
* Added transformation templates and fix in skill interface
* Updated changelog
* Contributors: Bjarne Grossmann, RvmiLab, Francesco Rovida

1.0.1 (2019-05-22)
------------------
* Better debug output in world model and sync parameters when preempting.
* Fixed bug in skill preemption
* Minor fix for output
* Fixed SelectorStar processor
* Fix in hold conditions (now checked before ticking childrens)
* Better output of conditions and reverted 2 changes causing problems.
* Serial processor preempting when switching. Skill call onEnd function when stopping.
* Avoiding crash when an object in the BB is removed from the world model
* Contributors: Bjarne Grossmann, RvmiLab, Francesco Rovida

1.0.0 (2019-04-30)
------------------
* Merge pull request `#11 <https://github.com/RVMI/skiros2/issues/11>`_ from matthiashh/matthias-devel-pycodestyle
  Matthias devel pycodestyle
* Merge branch 'develop' into matthias-devel-pycodestyle
* Moved process preempt in skill executor. Preempting correctly a skill when hold condition fails.
* Selector preempts correctly running skills when switching
* Added hold conditions to continuously evalutate a condition holds while ticking a skill
* Added hold conditions check on tick
* Removed old BSD tags
* Merge branch 'develop' into matthias-devel-pycodestyle
* Removed match blackboard feature.
* Merged 'develop' into 'pycodestyle'.
* Sync skill's parameters with world model before execution
* Finish plan-in-the-tree
* Applied pycodestyle (pep8) with "--aggresive"
  This should fix most violations of pep8.
  Excemptions were made for manually structured code through tabs.
* Bug-fix with tree ending in failure state and added info messages at skill manager boot
* Keeping root in running state when tree is Idle (avoid GUI to get stucked)
* Bug-fix with parallel execution of same type of skill
* Removed old BSD headings and minor bug-fixes
* Extended Element hasProperty function and fixed skill manager output on preemption and execution failures.
* Visualizing blackboard data when a BT crashes. Limited property output print length, to avoid overloading the screen.
* Added timeout to kill blocked BTs. Minor improvements on GUI
* GUI improvements: BT execution visualization and more intuitive interface
* Bug-fix in sequential processor and in Element addRelation
* Updated version and changelog file
* Cleaned dependencies
* Changed implementation of sequential processor
* Contributors: Francesco Rovida, Matthias Mayr, RvmiLab, Francesco Rovida

0.1.0 (2018-09-27)
------------------
* Bug-fix in GUI. Changed Idle status handling in BT Parallel processors.
* Fixed bug when preempting skills and with skill layer interface.
* Added properties for simple access in skill layer interface
* Finish wm-restructure
* Improved GUI and interfaces
* added hasRelation method to Element class. Fixed world model get_relation to work with subproperties. Other minor cleaning.
* Bug-fix with visitor skill and chained remappings
* Override __call_\_ operator in skills for easier scripting. Automatically updating elements when adding to wm. Passing parameters to skills before exapanding.
* Added DictDiff utility. Fix in world element. Extended addChild function for skills
* Changed world model interfaces methods to snake case
* Completed and tested implementation of wm contexts
* Cleaned obsolete dependencies on and launch files for skiros2_resource manager
* Removed obsolete package skiros_resource. Added Property msg and changed WmElement And WmMonitor msgs. Changed serialization accordingly.
* Cleaner implementation of skill's description modification
* Couple of fixes to specification of parameters
* Now a Idle state is handled as a Running (doesn't stop the task execution)
* Fixed selector (returns Idle)
* Improvments on instanciation of skills.
* Now if a specific label (= implementation) is specified for a skill wrapper only that implementation is considered for instanciation.
* Completed correct skill preemption
* Bug fix on preemption of skills.
* Bug fix in remapping and several bug-fix on the GUI.
* Fixed bug when creating chained remappings of parameters.
* Decreased output verbosity on GUI
* Simplified and cleaned memorization and print of skills' progresses. Now also printing the skill's parent name for clearer reading.
* Added time from start to skill progress.
* Tested multiple task execution/stop. Now skill manager can instantiate new skills when available ones are already running.
* Improved debug output on GUI.
* Bugfix to print out correctly trace. Clearing correctly the children when connecting a new instance to a wrapper.
* Further improvements to BT execution rate: decreased queries to wm and added interface to update only element properties. Bug fix on skills reset.
* Improvements to increase the BT ticking speed.
* Patch to always refresh WM view when starting the GUI
* Modified skill manager to be able to execute multiple BTs at a time.
* Minor fix in skill managers discovery
* Simplified and improved skill managers discovery mechanism
* Optimized GUI
* Optimized PDDL problem generation. Task planner returns success if replan is empty. Adding relation hasTemplate to all elements with a template. Added utility functions to world element and pddl interface.
* Increased ticking rate of BT to 50hz
* Added selector star (skipping failures)
* Fixed NoFail decorator. Now task planner returns success when no skills have to be executed.
* Bug-fix, now unlocking wm when preempted!
* Implemented lock/unlock of world model
* Implemented correctly preemption using visitors
* Updated function resolveElement
* Changes to discrete reasoners for better integration.
* Fixed skills autoparametrization
* Implemented replanning. Removed printouts on world model. Other minor improvements.
* Not resetting description when changing wrapper instance.
* Added unset function to params and removing parameter from map while merging if param is not specified.
* Added Enforce processor
* Temp
* Changed access to SkillWrappers to print output correctly. Updates on skill manager interface
* Feature: Progress output
* Added progress output
* Merge branch 'master' into master
* Patched incongruence in final skill state. To test better!
* Better fix for bug when skills return false on start
* Fixed bug when skills return false on start
* Fixed bug in skill mgr interface
* Fixed bugs: checking ontology before loading, error when re-starting skill mgr, setting default prefix in skill mgr
* Changed parameter types (`#10 <https://github.com/RVMI/skiros2/issues/10>`_)
  * Changed skill tab: added system parameters as mandatory and added empty field in optional parameters
  * Bug-fix in skill reset and adding element to world model.
  * Changed base parameters types. Removed World and Config became Required.
  * Added gui launch in world_model_server.launch
  * Fix avoiding gui crashing when a parameter is not specified
  * Removed System from ParamTypes
  * Setting name of the scene on the GUI according to init_scene parameter
* Bug-fix in skill reset and adding element to world model.
* Bug-fixes for turtlesim launch and tf publishing when loading a scene (`#8 <https://github.com/RVMI/skiros2/issues/8>`_)
* Bug-fix in skill mgr naming 2
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
* Bug-fix in skill manager naming
* Finish WP4_gui
  # Conflicts:
  #	skiros2_world_model/src/skiros2_world_model/ros/world_model_server.py
* Added sequential operator and made it default.
* Skill in success state are no more bypassed by visitor (are executed again). Other minor changes on output
* Bug-fix in world model load. On boot, skill manager remove old skill from the scene before adding new ones.
* Extended property condition with support for different operators. Added loop decorator. Minor change in world model server init.
* 0.0.2
* Changelogs added
* Decreased ourput verbosity of skill manager
* Bug-fix when using a namespace. Implemented simple policy for wm to stay up until clients are correctly disconnected.
* Added support to run skiros under a ROS namespace. Updated readME.
* Merge pull request `#1 <https://github.com/RVMI/skiros2/issues/1>`_ from RVMI/master
  Added robot discovery mechanism and execution monitor
* Updated monitor output
* Added possibility to set a callback on skill manager monitor topic. Task manager republish all skill mgrs monitor output to its own monitor.
* Finish WP5_task_feedback (preliminary)
  # Conflicts:
  #	skiros2_common/src/skiros2_common/core/abstract_skill.py
* Added progress message and publisher
* new visitor to expand bt and retrieve skill sequence used to monitor task progress
* new visitor to expand bt and retrieve skill sequence used to monitor task progress
* First commit
* Contributors: Bjarne Grossmann, DavidWuthier, Francesco Rovida, Francesco Rovida, ipa-led
