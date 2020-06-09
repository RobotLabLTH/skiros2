^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package skiros2_world_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-06-09)
------------------
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Merge pull request `#30 <https://github.com/RVMI/skiros2/issues/30>`_ from RVMI/python3-rdflib
  Conditional dependencies for rdflib
* add conditional dependencies for rdflib and port to format 3
* Fix crash when saving scene to new folder (issue `#28 <https://github.com/RVMI/skiros2/issues/28>`_)
* Fix to `#24 <https://github.com/RVMI/skiros2/issues/24>`_
* Merge remote-tracking branch 'rvmi/develop' into develop
* Merge branch 'develop' into develop-rss
  # Conflicts:
  #	skiros2/scripts/install_fd_task_planner.sh
* Merge remote-tracking branch 'origin/rvmi/develop' into develop
* Contributors: Francesco Rovida, Matthias Mayr, emmanuelkring, francesco

1.0.4 (2020-03-04)
------------------
* Keeping the order in get_recursive, so that root is first and children come afterward.
* Merge pull request `#23 <https://github.com/RVMI/skiros2/issues/23>`_ from RVMI/py3-fixes
  Py3 fixes
* Make comparisons work with python3 and convert dict_keys to list before indexing
* Merge pull request `#22 <https://github.com/RVMI/skiros2/issues/22>`_ from RVMI/world_model/python3-fixes
  World model/python3 fixes
* Python3 fixes in skiros2_world_model
* Fix on GUI shortcuts and Element hasRelation function.
* Merge pull request `#20 <https://github.com/RVMI/skiros2/issues/20>`_ from RVMI/emk/fixes
  Emk/fixes
* Fix bug in _get_dataType making the type not print on error
* Moved wmi to base skill class. Added function to determine if an element id refers to a template or an instantiated object.
* Added more documentation. Removed obsolete files. Moved generated files (e.g. logs and plans) into ~/.skiros folder. Simplified planner installation, that now doesn't modify system path.
* Started world model interface doc fix.
* Finish improved-debugging-tools
* 1.0.3
* Contributors: Emmanuel Kring, Francesco Rovida

1.0.2 (2019-08-26)
------------------
* Merge branch 'master' into develop
* Printing SLOW tag to primitives running with a period above 40msec.
* Updated changelog
* Contributors: RvmiLab, Francesco Rovida

1.0.1 (2019-05-22)
------------------
* Better debug output2
* Better debug output in world model and sync parameters when preempting.
* Avoiding crash when an object in the BB is removed from the world model
* Contributors: RvmiLab, Francesco Rovida

1.0.0 (2019-04-30)
------------------
* Ignoring skiros:hasTemplate relations in get_individual
* Merge pull request `#11 <https://github.com/RVMI/skiros2/issues/11>`_ from matthiashh/matthias-devel-pycodestyle
  Matthias devel pycodestyle
* Merge branch 'develop' into matthias-devel-pycodestyle
* Removed old BSD tags
* Merge branch 'develop' into matthias-devel-pycodestyle
* Removed match blackboard feature.
* Merged 'develop' into 'pycodestyle'.
* Sync skill's parameters with world model before execution
* Finish plan-in-the-tree
* Applied pycodestyle (pep8) with "--aggresive"
  This should fix most violations of pep8.
  Excemptions were made for manually structured code through tabs.
* Fixed rare concurrency error on world model query
* Bug-fix in element removal
* Wm updates cached elements instead of deleting, for better performances
* Updated version and changelog file
* Cleaned dependencies
* Updated setup.py for automatic installation of dependencies. Updated readme
* Contributors: Francesco Rovida, Matthias Mayr, RvmiLab, Francesco Rovida

0.1.0 (2018-09-27)
------------------
* Bug-fix in update properties function
* Bug-fix in GUI. Changed Idle status handling in BT Parallel processors.
* Finish wm-restructure
* Improved GUI and interfaces
* added hasRelation method to Element class. Fixed world model get_relation to work with subproperties. Other minor cleaning.
* Bugfix for adding element without label to a context
* Added default filename to load/save contexts.
* Added parameter to load automatically contexts on wm. Fix in context save function.
* Bug-fix and now removing an element twice doesn-t generate an error any more
* 2 bug-fixes on world model
* Bug-fixes
* Changed world model interfaces methods to snake case
* Bug-fix and minor edits
* Completed and tested implementation of wm contexts
* All functions in world model ported to snake case
* Started restructuring
* Removed obsolete package skiros_resource. Added Property msg and changed WmElement And WmMonitor msgs. Changed serialization accordingly.
* Bug fix in remapping and several bug-fix on the GUI.
* Bug fix in world model interface, when not using cache
* Clearing cache when on World model interface when out of sync with server
* World model now publishes changes of relations. World model interface erases cache of outdated objects.
* Revert some changes.
* Further improvements to BT execution rate: decreased queries to wm and added interface to update only element properties. Bug fix on skills reset.
* Improvements to increase the BT ticking speed.
* Patch to always refresh WM view when starting the GUI
* Fix to a possible concurrency issue when using getRecursive
* Completed interface for dynamic sync with wm
* Optimized GUI
* Optimized PDDL problem generation. Task planner returns success if replan is empty. Adding relation hasTemplate to all elements with a template. Added utility functions to world element and pddl interface.
* Patch on ontology query parse error.
* Reduced verbosity of resolveElements2 function
* Fixed NoFail decorator. Now task planner returns success when no skills have to be executed.
* Implemented lock/unlock of world model
* Extended removeElement function to handle id string in input
* Updated function resolveElement
* Changes to discrete reasoners for better integration.
* Added guard in WM against relations with keys not in ontology. Improvement to pddl domain generation. Bug fix in parameters merging. Removed some debug msgs.
* Checking abstract relations when parameterizing skills in BT
* Fixed skills autoparametrization
* Implemented replanning. Removed printouts on world model. Other minor improvements.
* Removed debug print
* Setting ids in addElements function
* Temp
* Changed access to SkillWrappers to print output correctly. Updates on skill manager interface
* Added addElements function to world model interface
* Fixed bugs: checking ontology before loading, error when re-starting skill mgr, setting default prefix in skill mgr
* Changed parameter types (`#10 <https://github.com/RVMI/skiros2/issues/10>`_)
  * Changed skill tab: added system parameters as mandatory and added empty field in optional parameters
  * Bug-fix in skill reset and adding element to world model.
  * Changed base parameters types. Removed World and Config became Required.
  * Added gui launch in world_model_server.launch
  * Fix avoiding gui crashing when a parameter is not specified
  * Removed System from ParamTypes
  * Setting name of the scene on the GUI according to init_scene parameter
* Setting name of the scene on the GUI according to init_scene parameter
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
* Fixed bug in instanciate function. Fixed bug in the gui's add and remove object buttons.
* Fixed bugs with elements caching
* GUI update on wm changes
* Bug-fix
* Checking file existence before loading scene.
* Bug-fix in world model load. On boot, skill manager remove old skill from the scene before adding new ones.
* Extended property condition with support for different operators. Added loop decorator. Minor change in world model server init.
* 0.0.2
* Changelogs added
* Bug-fix on world model getIndividual. Removed spaces from world model.py
* Bug-fix in getIndividuals function
* Added getIndividuals function to ontology interface
* Bug-fix when using a namespace. Implemented simple policy for wm to stay up until clients are correctly disconnected.
* Added support to run skiros under a ROS namespace. Updated readME.
* Skill's config params are no more communicated to the wm.
* First commit
* Contributors: Bjarne Grossmann, DavidWuthier, Francesco Rovida, Francesco Rovida, ipa-led
