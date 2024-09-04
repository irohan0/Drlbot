# DRL_Navigation_Robot_ROS2_Foxy

## Project Code - https://drive.google.com/drive/folders/1JqWMuSiqdxoVWGCTgoOkkhHtl17IVnZW?usp=sharing

## Project Report - https://drive.google.com/file/d/18x_Y4vA9kz1PsQ4C-PFyA92gKNaFRlzq/view?usp=sharing

*ABSTRACT* - In this study, a mobile robot equipped with a LIDAR sensor is trained to navigate to random goal points in a simulated environment while avoiding obstacles. The training, which was done in ROS Gazebo 11 with PyTorch, demonstrated the robot's learning skills, and was tested using ROS2 Foxy. TD3, which is based on the Deep Deterministic Policy Gradient (DDPG) architecture, uses an actor network to select actions and dual critic networks to reduce Q-value overestimate. The training procedure includes rewards for forward mobility and penalties for being near to obstacles. Tensorboard is used to monitor training progress and provide real-time visualization of the robot's learning process. This application demonstrates the potential of DRL to improve autonomous robot navigation. The robot's navigation accuracy of around 82% indicates the efficacy of the TD3 algorithm in improving autonomous navigation capabilities.

 Deep Reinforcement Learning for mobile robot navigation in ROS Gazebo 11 simulator. A robot is trained to navigate through a simulated environment, avoiding obstacles, to a randomly chosen destination point using a Twin Delayed Deep Deterministic Policy Gradient (TD3) neural network. The robot receives a goal in polar coordinates when its LIDAR (Light Detection and Ranging) sensor detects an obstacle. trained using PyTorch in the ROS Gazebo 11 simulator. conducted tests with ROS2 Foxy on Ubuntu 20.04 using PyTorch 1.10 and Python 3.8.10.

TD3 Network Implementation :

The TD3 algorithm (refer Algorithm 1 IN REPORT) uses an actor-critic architecture, in which the actor network decides which actions the robot should perform, and two critic networks analyze the state-action pairings to choose the lowest Q-value, resulting in more reliable estimations. The training goal is to enable the robot to efficiently travel to the goal spots while avoiding obstacles. To accomplish this, the reward function is designed to promote forward movement while penalizing proximity to obstacles. Specifically, the robot receives positive rewards for linear motion and negative rewards for approaching any obstacle closer than one meter.


![image](https://github.com/user-attachments/assets/f035ffe4-7b25-4e39-a878-5d1067263868)


The Robot and The Evironment :

We are trying to "find the optimal sequence of actions that lead the robot to a given goal”. There are two things to consider — the action and the environment that the action reacts to. In a mobile robot setting, it is quite easy to express the action in a mathematical form. It is the force applied to each actuator for the controllable degree of freedom. To put it simply, it is how much we want to move in any controllable direction.

    a = (v, ω)
    s = (laser_state + distance_to_goal + theta + previous_action)
    
- a is tuple action , v is translational velocity, ω is angular velocity
- s is state, laser_state are distances to an obstacle at each 9-degree interval within a 180-degree range in front of a robot from LIDAR sensor, theta is angles between the robot heading and the heading towards the goal

Reward :

if robot_reach_the_goal:

    r = 100
elif collision:

    r = -100.0
else:

    r = v - |ω| - r3  // r3 = (1 - smallest distance of robot to obstacles) if that distance < 1m else r3 = 0
    
r is the reward for each time step,
The idea behind it is that the robot needs to realize that it should be moving around and not just sitting in a single spot. By setting a positive reward for linear motion robot first learns that moving forward is good and rotating is not.Additionally, we add the term r3 which is calculated by our lambda function. This gives an additional negative reward if the robot is closer to any obstacle than 1 meter.

Training environment :

<p align="center">
    <img width=50% src="https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy/blob/main/Training_env.png">
</p>

## Installation
Main dependencies: 

* [PyTorch](https://pytorch.org/get-started/locally/)
* [Tensorboard](https://github.com/tensorflow/tensorboard)
* [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

```shell
$ sudo apt install python3-colcon-common-extensions
$ sudo apt install ros-foxy-gazebo-ros-pkgs
$ sudo apt install ros-foxy-xacro
```
Clone the repository :

```shell
$ git clone https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy.git
$ cd DRL_Navigation_Robot_ROS2_Foxy
```
Compile the workspace:

```shell
$ source /opt/ros/foxy/setup.bash
$ colcon build
$ source install/setup.bash
```
Training :
```shell
$ ros2 launch td3 training_simulation.launch.py
```
monitor the training process by tensorboard. Open the new terminal:

```shell
$ tensorboard dev upload --logdir     './src/td3/runs/train/tensorboard'
```
![image](https://github.com/user-attachments/assets/84a3d3c7-fd8a-4617-99dc-76fadf36a486)
![image](https://github.com/user-attachments/assets/93724b55-fffc-467b-a894-9715165820ba)
![image](https://github.com/user-attachments/assets/f3ac2b98-ff4c-40f9-a13a-334eb2179a44)


Training example :

<p align="center">
    <img width=70% src="https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy/blob/main/Training_example.gif">
</p>

Testing :
```shell
$ ros2 launch td3 test_simulation.launch.py
```
Test example :

<p align="center">
    <img width=70% src="https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy/blob/main/Test_example_env1.gif">
</p>

Deep reinforcement learning (DRL) for autonomous robot navigation utilizing the Twin Delayed Deep Deterministic Policy Gradient (TD3) method has shown promise in improving robotic capabilities. The robot effectively learns to navigate to random objective spots while avoiding obstacles in a simulated environment, with a navigation accuracy of around 82%. This result demonstrates the effectiveness of the TD3 method, notably its technique of employing an actor network for action decision and dual critic networks to combat Q-value overestimation. The training process, enabled by PyTorch and carried out in the ROS Gazebo 11 simulator, is efficiently visualized and monitored using Tensorboard, ensuring real-time feedback and progress monitoring. This work not only contributes to the field of autonomous robotics but also sets a foundation for future research and development in DRL-based navigation systems.
