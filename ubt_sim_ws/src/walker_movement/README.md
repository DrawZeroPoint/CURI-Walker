# Movement helpers

This package provides helpers for moving the limbs of the walker robots.

It provides services and actions for controlling the arms and the hands of the robot.

## Launching

You can launch the helpers by launching helpers.launch

It will launch the actions:
* /walker/move_helper_left_arm/move_to_joint_pose
* /walker/move_helper_right_arm/move_to_joint_pose
* /walker/move_helper_left_arm/move_to_ee_pose
* /walker/move_helper_right_arm/move_to_ee_pose
* /walker/hand_helper_left/grasp
* /walker/hand_helper_right/grasp
* /walker/move_helper_left_arm/follow_ee_pose_trajectory
* /walker/move_helper_right_arm/follow_ee_pose_trajectory
* /walker/dual_arm_control/follow_ee_pose_trajectory
* /walker/dual_arm_control/move_to_ee_pose
* /walker/dual_arm_control/move_to_ee_pose_mirrored
* /walker/dual_arm_control/move_to_joint_pose



And the services:
/walker/move_helper_left_arm/get_joint_state
/walker/move_helper_right_arm/get_joint_state

By specifying enable_legs:=true the same services will also be available for the
two legs, under /walker/move_helper_left_leg, /walker/move_helper_right_leg and /walker/dual_leg_control

You can also use the all_deps.launch launcher to start the helpers together with
dependencies such as the robot controllers or the gait module.

## Arms and Legs

The package provides actions and a service for controlling arms and legs. To enable
the control of the legs on the all_deps.launch and helpers.launch files you must
specify legs_control:=true. Note that the leg control should not be run while the
gait module is running.

### get_joint_state service

The service allows to get the current joint state for the arm. No parameters are required.

### move_to_joint_pose action

The action allows to move the arm to a joint pose.


### move_to_ee_pose action

The action allows to move the arm end effector to a cartesian pose.
The end effector link can be selected using the end_effector_link parameter
You can specify the frame of the goal pose using the header of the PoseStamped

### follow_ee_pose_trajectory

This action commands the arm end effector to follow a cartesian-space trajectory.
The poses the end effector has to pass through are specified in the poses parameter.
You can use the frame_id to perform relative movements.
All frame conversions are performed before the trajectory execution.

The times at which each pose has to be reached are specified in the times_from_start parameter.
YOu can specify the end effector frame to use in the end_effector_link parameter, leaving this
empty will result in the use of the default end effector (see .action file for details).

An example of the use of this action can be seen in scripts/leg_trajectory_test.py

### dual_arm_control/move_to_ee_pose

This action allows to perform the move_to_ee_pose task on both arms simultaneously.
You parameters are the same as move_to_ee_pose, just repeated for left and right.

**NOTE:** the planner is not aware of the simulatenous movement of the two arms. Consequently
when planning for one arm it will consider the other one to be at its original position.
This means it may generate two plans for the arms that collide with each other. At the same time,
it will not be aware of the fact the other arm has moved from its original location,
meaning it may report false collisions and fail to generate a plan.

### dual_arm_control/move_to_ee_pose_mirrored

This action performs the same task as dual_arm_control/move_to_ee_pose, but the
planning is only performed for the left arm, and then it is mirrored to generate
a plan for the right one. This ensures the two arms follow simmetrical paths.

**NOTE:** The same considerations about collisions as in dual_arm_control/move_to_ee_pose apply here as well

### dual_arm_control/move_to_joint_pose

This performs the same task as move_to_joint_pose, but on the two arms at the same time.
You can generate mirrored plans as in move_to_ee_pose_mirrored by setting the mirror parameter.

**NOTE:** The same considerations about collisions as in dual_arm_control/move_to_ee_pose apply here as well


### dual_arm_control/follow_ee_pose_trajectory

This action performs the same task as the regular follow_ee_pose_trajectory, but on two limbs at the same time.

**NOTE:** no planning is performed in this action. Therefore not collision checking is performed.


### Legs control

As specified before, to control the legs you can use the legs_control parameter in the launch files.
The same services a for the arms will be made available for the legs. The will appear as the action and services:

* /walker/move_helper_left_leg/move_to_joint_pose
* /walker/move_helper_right_leg/move_to_joint_pose
* /walker/move_helper_left_leg/move_to_ee_pose
* /walker/move_helper_right_leg/move_to_ee_pose
* /walker/move_helper_left_leg/follow_ee_pose_trajectory
* /walker/move_helper_right_leg/follow_ee_pose_trajectory
* /walker/dual_leg_control/follow_ee_pose_trajectory
* /walker/dual_leg_control/move_to_ee_pose
* /walker/dual_leg_control/move_to_ee_pose_mirrored
* /walker/dual_leg_control/move_to_joint_pose
* /walker/move_helper_left_leg/get_joint_state
* /walker/move_helper_right_leg/get_joint_state


## Hands: grasp action

You can specify the grasp type using the grasp_type parameter. You can use this parameter
also to open the hand by using GRASP_TYPE_OPEN

# Examples: Control-only tasks

## Task 2: Grasp Can

The script scripts/graspCan.py performs the task number 2, it grabs and raises the can number 1 from the table

To use it you will first need to launch helpers.launch and moveit.launch (from walker_webots_hardware_interface) in separate terminals.
Then graspCan can be started as:

```
rosrun walker_movement graspCan.py _can_number:=1
```

## Task 6: Push Cart

The script scripts/pushCart.py performs task number 6, it grabs the cart and pushes it forward

To use the script you will first need to launch all_deps.launch:

```
roslaunch walker_movement all_deps.launch
```

The you can launch the script with:

```
rosrun walker_movement pushCart.py
```

## Task 10: Open Refrigerator

The script scripts/openFridge.py performs task number 10, it opens the the refrigerator without navigation or vision

To use the script you will first need to launch all_deps.launch:

```
roslaunch walker_movement all_deps.launch
```

The you can launch the script with:

```
rosrun walker_movement openFridge.py
```

## Task 14: Grasp Box and Walk Backwards.

This task can be executed in two phases.

The first one, grasping the box, can be ran with the command:

```
rosrun walker_movement liftBox.py
```

The second one, the walking backwards with the grasped box, can be ran with:

```
rosrun walker_walk mat_pyth.py
```