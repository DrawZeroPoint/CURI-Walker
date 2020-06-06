# CURI-Walker

Team CURI's solution for World Artificial Intelligence Conference (WAIC) 2020 Walker challenge 
competition.

# Pre-requests

Ubuntu 18.04, ROS Melodic, Webots R2020a

# Project structure

## ubt_sim_ws

The catkin workspace for development. Two official packages named `example` and 
`ubt_core_msgs` are exist in `src/`. At the end of this match, we may establish
a single package to fulfill all tasks required.

## walker_install

Official pre-built gait package.

## walker_model

Official Webots model and controllers. **No modification should be carried out 
in this part**.

# Installation

1. In `~`, `git clone https://github.com/DrawZeroPoint/CURI-Walker.git`

2. In new terminal, `source ~/CURI-Walker/walker_install/setup.bash`, and then 
   `cd CURI-Walker/ubt_sim_ws/ && catkin build`

3. Add `source ~/CURI-Walker/walker_install/setup.bash` and `source ~/CURI-Walker/ubt_sim_ws/devel/setup.bash`
   into `~/.bashrc`

# Execution

1. Run `roscore`

2. Start Webots, open `WAIC.wbt` in `walker_model/worlds/WAIC.wbt`

3. Launch 

# Dev Guide

## Strategy

We may follow a distributed development scheme to push forward, i.e., each
developer takes charge of one or more tasks, whereas at the final stage all
contributions are composed into one package which runs seamlessly to meet
the rules.

## Branch

All developers should maintain their code in individual branch 
distinguishable by some ubiquitous prefix, like `dzp-`, and also highlight the
task # afterwards, hence the branch name could be like `dzp-task1`, meaning 
that this branch is tailored for solving 1st task.

The `master` branch should serve as a final archive of the development, and
any in-dev branch mush be reviewed by the team leader or the whole team 
before it could be merged into `master`.

## Dev language

The developers may either using C++ or Python, or even both for 
coding the ROS package.

## Code style



# Cheat sheet

## ROS topics from WebotsAPI

| Name                            | Type                         |
|---------------------------------|------------------------------|
| /Leg/DesiredJoint               | sensor\_msgs/JointState      |
| /walker/Leg/MeasuredJoint       | sensor\_msgs/JointState      |
|                                 |                              |
| /sensor/head\_imu               | sensor\_msgs/Imu             |
| /sensor/orientus\_imu           | sensor\_msgs/Imu             |
| /sensor/camera\_imu             | sensor\_msgs/Imu             |
|                                 |                              |
| /sensor/ft/lankle               | geometry\_msgs/WrenchStamped |
| /sensor/ft/lwrist               | geometry\_msgs/WrenchStamped |
| /sensor/ft/rankle               | geometry\_msgs/WrenchStamped |
| /sensor/ft/rwrist               | geometry\_msgs/WrenchStamped |
|                                 |                              |
| /walker/camera/bottomDepth      | sensor\_msgs/Image           |
| /walker/camera/bottomRGB        | sensor\_msgs/Image           |
| /walker/camera/doubleLeftRGB    | sensor\_msgs/Image           |
| /walker/camera/doubleRightRGB   | sensor\_msgs/Image           |
| /walker/camera/headDepth        | sensor\_msgs/Image           |
| /walker/camera/headRGB          | sensor\_msgs/Image           |
| /walker/camera/topDepth         | sensor\_msgs/Image           |
| /walker/camera/topRGB           | sensor\_msgs/Image           |
|                                 |                              |
| /walker/head/controller         | ubt\_core\_msgs/JointCommand |
| /walker/head/joint\_states      | sensor\_msgs/JointState      |
| /walker/leftHand/controller     | ubt\_core\_msgs/JointCommand |
| /walker/leftHand/joint\_states  | sensor\_msgs/JointState      |
| /walker/leftLimb/controller     | ubt\_core\_msgs/JointCommand |
| /walker/leftLimb/joint\_states  | sensor\_msgs/JointState      |
| /walker/rightHand/controller    | ubt\_core\_msgs/JointCommand |
| /walker/rightHand/joint\_states | sensor\_msgs/JointState      |
| /walker/rightLimb/controller    | ubt\_core\_msgs/JointCommand |
| /walker/rightLimb/joint\_states | sensor\_msgs/JointState      |
|                                 |                              |
| /walker/ultrasound/leftBack     | sensor\_msgs/Range           |
| /walker/ultrasound/leftFront    | sensor\_msgs/Range           |
| /walker/ultrasound/middleBack   | sensor\_msgs/Range           |
| /walker/ultrasound/rightBack    | sensor\_msgs/Range           |
| /walker/ultrasound/rightFront   | sensor\_msgs/Range           |


