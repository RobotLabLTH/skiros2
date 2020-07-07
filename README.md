#  Skill-based robot control platform for ROS V2.0 (SkiROS2)

SkiROS V2.0 is a platform to create complex robot behaviors by composing _skills_ - modular software blocks - into [behavior trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)).

Robots coordinated with SkiROS can be used in partially structured environments, where the robot has a good initial understanding of the environment, but it is also expected to find discrepancies, fail using initial plans and react accordingly.

SkiROS offers the following features:  

* A framework to organize the robot behaviors within modular skill libraries
* A reactive execution engine based on Behavior trees
* An integration point for PDDL task planning using the "task planning" skill
* A semantic database to manage environmental knowledge

For a full introduction and tutorials refer to the [wiki](https://github.com/RVMI/skiros2/wiki)

### Acknowledgements

This platform has been developed at the [RVMI lab](http://www.rvmi.aau.dk/), with the support of the European Horizon2020 project [Scalable4.0](https://www.scalable40.eu/).

## Install

To use SkiROS you must have [ROS](https://wiki.ros.org/ROS/Installation) installed on your machine.  

You must include this package in your catkin workspace and compile with catkin. Also, it is highly recommended to include in the catkin workspace the [base skill set](https://github.com/RVMI/skiros2_std_lib).

SkiROS is compatible with Ubuntu 16.04/ROS Kinetic, Ubuntu 18.04/ROS Melodic and Ubuntu 20.04/ROS Noetic, python 2 and 3.
  
Dependencies can be installed with the python tool:  

> cd skiros2  
> pip install -r requirements.txt --user  

Optionally, if you want to use the task planning skill provided in the standard library, install the fast downward planner with the following script:

> cd skiros2/scripts  
> ./install_fd_task_planner.sh  

When asked for the install folder, you can insert a location of your preference or just leave the default.

## Launching the system

See the [skiros2_template_lib](https://github.com/RVMI/skiros2_template_lib) for a minimal example on how to launch the system (also useful to create a new library from scratch).

See the [skiros2_examples](https://github.com/RVMI/skiros2_examples) for more elaborated examples.

## Doxygen documentation

To generate the doxygen documentation, first install Doxygen:

> sudo apt-get install -y doxygen

Install doxypypy for better compatibility with python doc-strings:

> pip install doxypypy --user

Then, in the top level directory, generate the documentation:

> doxygen Doxyfile

Finally, you can view it with for example firefox:

> firefox skiros2_doc/html/index.html
