# Movement helpers

This package provides helpers for moving the limbs of the walker robots.

It provides services and actions for controlling the arms and the hands of the robot.

## Launching
You can launch the helpers by launching helpers.launch

It will launch the actions:
/walker/move_helper_left_arm/move_to_joint_pose
/walker/move_helper_right_arm/move_to_joint_pose
/walker/move_helper_left_arm/move_to_ee_pose
/walker/move_helper_right_arm/move_to_ee_pose
/walker/hand_helper_left/grasp
/walker/hand_helper_right/grasp

And the services:
/walker/move_helper_left_arm/get_joint_state
/walker/move_helper_right_arm/get_joint_state

## Arms
The package provides two actions and a service for controlling each arm.

### get_joint_state service
The service allows to get the current joint state for the arm. No parameters are required.

### move_to_joint_pose action
The action allows to move the arm to a joint pose.


### move_to_ee_pose action
The action allows to move the arm end effector to a cartesian pose.
The end effector link can be selected using the end_effector_link parameter
You can specify the frame of the goal pose using the header of the PoseStamped

## Hands: grasp action
You can specify the grasp type using the grasp_type parameter. You can use this parameter
also to open the hand by using GRASP_TYPE_OPEN

# An example
You can look at the graspCan.py node for an example on using this package.
It can be launched as:
```
rosrun walker_movement graspCup1.py _can_number:=1
```
