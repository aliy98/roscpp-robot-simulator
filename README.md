# roscpp robot simulator
This is a simple rospackage to simulate and control a robot.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=KP1wJ6qN2e8&t=25s
" target="_blank"><img src="http://img.youtube.com/vi/KP1wJ6qN2e8&t=25s/0.jpg" 
alt="youtube video" width="240" height="180" border="10" /></a>

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

The rosgraph would look like this:
![rosgraph](https://user-images.githubusercontent.com/65722399/141699520-a63f6a5f-624c-4cb6-8501-2af88397e1ba.png)

## Pseudocode
```
***
  controller node:
  
  while(1)
  {
    if reset command is subscribed
    {
      request reset position service
    }
    
    if there is no obstacle in front of robot
    {
      move robot forward with the user input speed
      if robot is getting close to the obstacle from the left side
      {
        stop the robot and turn it to the right side
      }
      
      if robot is getting close to the obstacle from the right side
      {
        stop the robot and turn it to the left side
      }
    }
    else
    {
      if robot is closer to the obstacle in the left side
      {
        while there is still obstacle in front of robot
        {
          if reset command is subscribed
          {
            request reset position service
          }
          turn robot to the right
        }
      }
      else
      {
        while there is still obstacle in front of robot
        {
          if reset command is subscribed
          {
            request reset position service
          }
          turn robot to the left
        }
      }
    }
  }

  
***
  command node:
  
  while(1)
  {
    print("enter robot speed or enter "r" to reset robot position:")
    scan("string")
    if("string" == "r") publish reset command
    else
    {
      speed = str2float("string")
      if(0<speed<2) publish speed
      else print("error")
    }
  }
  
```
