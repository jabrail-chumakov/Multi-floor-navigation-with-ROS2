Multi-floor navigation using ROS2
================================

This is a simulation of robot navigation in a multi-floor environment implemented in ROS2. It was done as a part of the Software Architecture and Robotics course and written in Python.

Presentation to this project
----------------------

You can find presentation to this project on the following link: [presentation](https://docs.google.com/presentation/d/1Mzx13TFX9f57MzjbZGBgpcDochgEmuTHe6oEpUslk6Q/edit#slide=id.p)

Also, if you have some problems with missing files, you can find the whole workspace on this [link](https://drive.google.com/drive/u/0/folders/1sHIAlIqJ8uGyvdpecEqLRhxajHQJ1a4W)

Installation
----------------------

To run this program you should first install the following:
- Install the [ROS 2 binary packages](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) as described in the official docs.
- Install the Nav2 packages using your operating system’s package manager:
  ```javascript
  sudo apt install ros-<ros2-distro>-navigation2
  sudo apt install ros-<ros2-distro>-nav2-bringup
  ```
  Note: `<ros2-distro>` is the name of your ROS2 version.
- Place all files from this GitHub repository in your working workspace's newly created `my_robot` folder (such as dev_ws/src/my_robot).
- Navigate to your workspace in terminal and write `colcon build`.

Nav2
-----------------------------

The Nav2 project is the spiritual successor of the ROS Navigation Stack. This project seeks to find a safe way to have a mobile robot move from point A to point B. It can also be applied in other applications that involve robot navigation, like following dynamic points. This will complete dynamic path planning, compute velocities for motors, avoid obstacles, and structure recovery behaviors. Nav2 uses behavior trees to call modular servers to complete an action. An action can be to compute a path, control effort, recovery, or any other navigation related action. These are each separate nodes that communicate with the behavior tree (BT) over a ROS action server. 

Exercise
-----------------------------

In order to run the program, you should enter the commands below in the following order:

```bash
$ ros2 launch my_robot my-robot.launch.py 
$ ros2 run my_robot test.py 
```
The first script should run Gazebo and RVIZ in the new windows. The second script will move the robot between floors. 
Note: Don't forget to build your workspace before writing these commands above.

Working principle of proposed solution
-----------------------------

The proposed solution is simple and efficient and works almost perfectly. The solution consists of several cases when the robot needs to complete a particular command. The proposed solution works as follows, initially the robot is placed on the first floor, and then moves to the worker to pick something from him. After that robot moves to a special elevator zone (similar to a real elevator). As soon as the robot reaches the special zone and stays there for a period of time, the robot has an opportunity to move to another floor (floor 0) and dynamically changes the currently loaded map for navigation. It is important to note that on each floor there is a special movement zone, reaching which the robot has the opportunity to reach another floor. Thus, the main goal of the task, to change the map upon entering another floor, was achieved. After reaching floor 0, the robot moves to pick up something from the map and then goes back to the elevator and moves to the last floor to hand over the delivery package to the doctor.

`changeMap(map_filepath)` - Requests a change from the current map to map_filepath's yaml.

Video Demonstration
-----------------------------

Below you can watch a demonstration of this assignment:

[<img src="https://user-images.githubusercontent.com/67557966/178591457-5f028d8b-1b60-413b-86c1-059195b7ca9b.png" width="80%">](https://youtu.be/GY2TLpTeGtY))

Possible improvements
-----------------------------

Despite the fact that this script works pretty well, there are still some moments that could be improved in future.

- Suppose the robot is manually moved to another floor by the user. In the future, the feature can be implemented to determine on which floor the robot is currently located after an external intervention. One of the possible options is to find distinctive features in the scene, by which the robot can determine which floor has such features.
- A more dynamic scene could be created with a moving elevator where the robot would have to enter and choose a command on which floor to drive. Definitely, it will affect performance and slow down the simulation since it's a computationally expensive solution.
- More floors can be considered. Additionally, it can be assumed that the robot doesn't know the individual map of each floor, which makes a project more interesting and realistic.
- Also, robot navigation with the keyboard (arrow keys) can be implemented.


