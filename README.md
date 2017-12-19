###  Skill based framework for ROS 2.0 (SkiROS2) from the RVMI lab, Aalborg University Copenhagen, Denmark

[www.rvmi.aau.dk(RVMI webpage)](http://homes.m-tech.aau.dk/francesco/skiros2_doc/)

Last update: 25/05/2017  

**Compatibility**: Has been tested with Ubuntu 16.04, ROS Kinetic and python 2.7.

**SkiROS** is a collection of ROS packages to develop, test and deploy behaviours for autonomous robots. Using SkiROS, the developer can split complex *tasks* into *skills*, that get composed automatically at run-time to solve goal oriented missions. The *skills* can themselves be divided into an arbitrary amount of submodules, called *primitives*. Moreover, the framework helps to manage the robot knowledge with the support of a shared semantic world model. 
Using *discrete reasoners*, it is possible to embed in the code reasoning routines with an high level of abstraction. 

The development process consist of two steps: specify the domain knowledge in a OWL ontology and develop the plug-ins. 

The ontology, coded in one or more .owl file, defines which data, concepts, relations and individuals are possible to store to and retrieve from the world model. 
The knowledge base can be extended from the developer with multiple OWL ontologies. 
It is possible to specify the workspace, by specifying the parameter ”skiros_wm/workspace_dir". 
All the OWL files found in the specified path are loaded from the system at boot. 
To create and edit ontologies, we suggest to use the GUI [**Protege_5.0**](http://protege.stanford.edu/download/protege/5.0/snapshots/). 


### Included packages 

* **skiros2:** holds launch files and handful script to install SkiROS dependencies. It is also the standard workspace directory to load ontology files and scenes. (meta-package)  
* **skiros2_common:** common use classes, Param, ParamHandler, NodeSpawner and other utilities.  
* **skiros2_msgs:** all ROS messages used in the system  
* **skiros2_world_model:** holds the world model node, C++ wrappers for ROS interfaces, utilities to treat ontologies and the base class for conditions and reasoners. It provides two databases:  
   1. the **world ontology**, an abstract description of the world structure (data, concepts, relations and individuals)  
   2. the **world scene**, the instance of the world  
* **skiros2_resource:** manager of hardware and primitives  
* **skiros2_skill:** manager of skills (primitive networks)
* **skiros2_task:** manager of task (skill sequences)

### Dependencies

pip install networkx==1.11 semanticnet flufl.enum rdflib wrapt

### Install

### Explore ontology

Use 

### Launching the system

### Doxygen documentation



