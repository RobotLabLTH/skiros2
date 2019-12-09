###  Skill based framework for ROS 2.0 (SkiROS2) from the RVMI lab, Aalborg University Copenhagen, Denmark

[RVMI webpage](http://www.rvmi.aau.dk/)

**Compatibility**: Has been tested with Ubuntu 16.04/ROS Kinetic and Ubuntu 18.04/ROS Melodic, with python 2.7.

The Skill-based platform for ROS V2 (SkiROS2) is a software designed to support programming of complex coordination schemes on robots equipped with several sensing and actuation functionalities.

Robots coordinated with SkiROS can be used in partially structured environments, where the robot has a good initial understanding of the environment, but it is also expected to find discrepancies, fail using initial plans and react accordingly.

In a nutshell, SkiROS provides:
* A framework to program skills with the extended Behavior Tree model
* A framework for automatically concatenate skills using PDDL task planning
* Services to manage the knowledge integration with the support of a semantic world model.
* The *discrete reasoners* to embed in the code qualitative conditions grounded on quantitative data.

The development process consist of two steps: specify the domain knowledge in a OWL ontology and develop the plug-ins.

For detailed info refer to the [wiki](https://github.com/RVMI/skiros2/wiki)

### Dependencies

> roscd skiros2/..\
> pip install -r requirements.txt --user

### Task planning

To run the task planning you should install the fast downward planner:

> roscd skiros2/scripts\
> ./install_fd_task_planner.sh

When asked for the install folder, you can insert a location of your preference or just leave the default.

### Explore ontology

To create and edit ontologies, we suggest to use the GUI [**Protege_5.0**](http://protege.stanford.edu/download/protege/5.0/snapshots/).
The ontology, coded in one or more .owl file, defines which data, concepts, relations and individuals are possible to store to and retrieve from the world model.
The core ontology can be found in the folder skiros2/owl. The knowledge base can be extended from the developer with custom OWL ontologies.
It is possible to specify the workspace, by specifying the parameter ”wm/workspace_dir". (see world_model_servel launch file)
All the OWL files found in the specified path are loaded from the system at boot.

### Launching the system

See the *skiros2_template_lib* for a minimal example on how to launch the system (also useful to create a new library from scratch).

See the *skiros2_test_lib* for more elaborated examples.

### Doxygen documentation

TODO


