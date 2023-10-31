[toc]

# Environment Configuration

## qpOASES

We use qpOASES to solve MPC，and MPC is used for trajectory tracking

qpOASES  solver installation :

```bash
cd qpOASES
mkdir build
cd build
cmake ..
sudo make
sudo make install
```

## patchwork

patchwork is used for ground segmentation. This planner accepts 3d point cloud information **Pointcloud2**. After the point cloud is segmented on the ground, the point cloud is projected to the 2d plane to construct a 2d grid map for planning.

Follow the official repository to download this package：[patchworl_github](https://github.com/LimHyungTae/patchwork)

## Simulation environment

The simulation environment uses the open source simulation environment of the CMU Robotics Institute. Users  need to install the relevant libraries and download the mesh file. We recommend to use the indoor environment to test the planner.

The environment have two versions for ros-melodic and ros-noetic. The package in this project is for ros-melodic.

[CMU-environment](https://www.cmu-exploration.com/)

# Quick Start

First , start the simulation environment :

```bash
roslaunch vehicle_simulator system_indoor.launch
```

Second, start the planner :

```bash
roslaunch ego_planner run_in_sim.launch
```

In rviz , use  **2d nav goal** to set the goal :

![ego example](imag/ego_example.gif)

If you can't see the imag in this file, you can see it in the folder "imag".



# Explanation of Parameter Adjustments

In the simulation environment provided by CMU, the car does not have a real physical engine. So the car can move strictly according to control commands (linear velocity v and angular velocity w), which will affect the parameter setting of the MPC controller.

The objective function of the MPC controller consists of two parts: tracking error and motion constraints. Users can adjust the following 4 parameters:

```xml
<!-- adjust in advanced_param.xml -->
<param name="MPC/v_max" value="1.8" type="double"/>
<param name="MPC/w_max" value="1.2" type="double"/>
<param name="MPC/omega0" value="1.0" type="double"/>
<param name="MPC/omega1" value="0.5" type="double"/>
```

`MPC/v_max` and `MPC/w_max` are related to the maximum speed and acceleration set during planning and can be set slightly larger than the maximum planning speed `max_vel` .

`MPC/omega0` and `MPC/omega1` are the weights for the tracking error and motion constraints mentioned above respectively:

1. A larger value of `omega0` makes the MPC solver tend to reduce the error more, but it may exceed the robot's kinematic limits.
2. A larger value of `omega1` makes the MPC solver tend to make the robot's motion smoother, but it increases the error between the reference trajectory.

In the simulation environment provided by CMU, it is suggested to increase `omega0` because the motion of the car in this simulation environment is not restricted.

However, in a simulation environment with a physics engine, it is recommended to increase `omega1` appropriately. Otherwise, it is easy to encounter phenomena where the angular velocity and linear velocity are too large, resulting in unsmooth motion of the robot.

Plan to provide a version of ego-planner with integrated SLAM and a simulation environment with a physics engine in a [branch](https://github.com/Dangko/ego-planner-for-ground-robot/tree/dev-work-with-slam_module). 
It will be upload  once I have time to organize it.



# Update Logs

- 2023.10.31 

  Fixed the bug of the car switching its front end.

  Fixed the bug in MPC parameter adjustment and added parameter interface and documentation.

  Added an indoor scene of the CMU simulation environment, allowing users to test directly without downloading.



# THANKS

We are very grateful for Fei Gao and the EGO-Planner he proposed. Our algorithm is implemented under their algorithm framework. If you are interested in our algorithm, you may see the origin algorithm in :

[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)

