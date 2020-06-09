^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package skiros2_task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-06-09)
------------------
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Merge remote-tracking branch 'rvmi/develop' into develop
* Merge branch 'develop' into develop-rss
  # Conflicts:
  #	skiros2/scripts/install_fd_task_planner.sh
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Contributors: Matthias Mayr, francesco

1.0.4 (2020-03-04)
------------------
* Merge pull request `#23 <https://github.com/RVMI/skiros2/issues/23>`_ from RVMI/py3-fixes
  Py3 fixes
* Change name of method iteritems in param handler
* Python3 fixes in skiros2_task
* Converted all enums from flufl.enum to standard enum. Removed dependency.
* Added SerialStar processor and deprecated Sequential (still there as an alias). Comments to task manager interface
* Added more documentation. Removed obsolete files. Moved generated files (e.g. logs and plans) into ~/.skiros folder. Simplified planner installation, that now doesn't modify system path.
* Finish improved-debugging-tools
* Printing clear error message when task planner is not found
* 1.0.3
* Contributors: Francesco Rovida, emmanuelkring, Francesco Rovida

1.0.2 (2019-08-26)
------------------
* Merge branch 'master' into develop
* Changed way to evaluate abstract conditions (now based on class).
* Updated changelog
* Contributors: Francesco

1.0.1 (2019-05-22)
------------------

1.0.0 (2019-04-30)
------------------
* Planner uses hold conditions
* Merge pull request `#11 <https://github.com/RVMI/skiros2/issues/11>`_ from matthiashh/matthias-devel-pycodestyle
  Matthias devel pycodestyle
* Merge branch 'develop' into matthias-devel-pycodestyle
* Removed old BSD tags
* Merge branch 'develop' into matthias-devel-pycodestyle
* Merged 'develop' into 'pycodestyle'.
* Finish plan-in-the-tree
* Applied pycodestyle (pep8) with "--aggresive"
  This should fix most violations of pep8.
  Excemptions were made for manually structured code through tabs.
* Updated version and changelog file
* Contributors: Francesco Rovida, Matthias Mayr, RvmiLab, Francesco Rovida

0.1.0 (2018-09-27)
------------------
* Bug-fixes in task planning action and support for multiple skill libraries
* Finish wm-restructure
* Improved GUI and interfaces
* Changed world model interfaces methods to snake case
* Simplified and improved skill managers discovery mechanism
* Optimized PDDL problem generation. Task planner returns success if replan is empty. Adding relation hasTemplate to all elements with a template. Added utility functions to world element and pddl interface.
* Fixed typo in task manager
* Fixed warn output
* Fixed NoFail decorator. Now task planner returns success when no skills have to be executed.
* Revert "Added preempt function in task manager interface"
  This reverts commit 529ab7c8d16b9dc2c78b1484d3d668f57e561151.
* Added preempt function in task manager interface
* Added guard in WM against relations with keys not in ontology. Improvement to pddl domain generation. Bug fix in parameters merging. Removed some debug msgs.
* All unicode in params converted to str type. Fixed issue in task manager that sometime was not mapping the right value to the parameter key.
* Added cap to allowed replans
* Fixed concurrency bug when sending fast commands to task manger
* Implemented replanning. Removed printouts on world model. Other minor improvements.
* Maybe a fix?
* Minor fixes
* Added script to quick test planning domains. Now differentiating between end and start effects on an heuristic.
* Bug-fix in task manager action server. Added support for property conditions on goals.
* Debug messages for planner removed
* Debug messages for planner added
* Bugfix: Task manager waits for server
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
* Changed base parameters types. Removed World and Config became Required.
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
* 0.0.2
* Changelogs added
* Bug-fix when using a namespace. Implemented simple policy for wm to stay up until clients are correctly disconnected.
* Merge pull request `#1 <https://github.com/RVMI/skiros2/issues/1>`_ from RVMI/master
  Added robot discovery mechanism and execution monitor
* Updated monitor output
* Added possibility to set a callback on skill manager monitor topic. Task manager republish all skill mgrs monitor output to its own monitor.
* Changed robot_description output
* Changed output of robot_description. Updated robot name in task mgr launch
* Hotfix: Finish WP5_task_feedback (preliminary)
* Moved discovery to skiros ns; fixed subscriber msg
* Finish WP5_robot_discovery (preliminary)
* Added robot description publisher
* Optimized some code;
  Added dummy sub and pub for robot discovery
* Bug-fix
* First commit
* Contributors: Bjarne Grossmann, Francesco Rovida, Francesco Rovida, ipa-led
