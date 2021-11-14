# roscpp robot simulator
This is a simple rospackage to simulate and control a robot.
## Installing and runnning 
Here is the instruction for using the package:
```bashscript
$ mkdir -p catkin_ws/src
$ cd src
$ git clone https://github.com/aliy98/roscpp_robot_simulator
$ cd ..
$ source /opt/ros/<distro>/setup.bash
$ catkin_make
$ source /devel/setup.bash
$ roscore &
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
in order to run the controller node, open a new terminal in the same directory and run the following commands:
```bashscript
$ source /devel/setup.bash
$ rosrun second_assignment controller_node
```
now robot starts to move in the circuit.

For initializing the command node also, in a new terminal run the following commands:
```bashscript
$ source /devel/setup.bash
$ rosrun second_assignment command_node
```
you can input new values for robot speed between 0 and 2 or reset the robot position.
