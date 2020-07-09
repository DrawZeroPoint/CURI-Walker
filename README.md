# CURI-Walker

Team CURI's solution for World Artificial Intelligence Conference (WAIC) 2020 Walker challenge
competition. [Source Code]

# Pre-requests

Ubuntu 18.04, ROS Melodic, Webots R2020a

# Project structure

## ubt_sim_ws

The catkin workspace for development. Two official packages named `example` and
`ubt_core_msgs` exist in `src/`. Besides that, we have a series of packages started
with `walker_` prefix that are tailored for this contest, with the exception of
the `hope` package, which is based on a prior work of our team.

A comprehensive introduction of the customized packages is as follows:

**hope**: The object detection package that could detect horizontal planes and 
objects on the plane given the point cloud of the scene. We use this in GraspCup,
PushCart, and OpenFridge tasks, which helps for locating the cup to grasp, the
cart, or the fridge.

**walker_brain**: The primary entrance for solving the tasks. Refer its README
for details.

**walker_description**: URDF and meshes for the Walker robot.

**walker_moveit_config**: MoveIt! configure files generated from walker description.

**walker_movement**: Primary control functionalists.

**walker_nav**: Primary navigation stack.

**walker_webots_hardware_interface**: Interface for interacting with the Webots
simulator.

## walker_install

Official pre-built gait package.

## walker_model

Official Webots model and controllers. **No modification should be carried out
in this part**.

# Installation

1. In `~`, `git clone --recrusive https://github.com/DrawZeroPoint/CURI-Walker.git`. You could manually re-create the ubt_sim_ws
   following this [ROS] instruction, and then move the contents back.

2. In new terminal, `source /opt/ros/molodic/setup.bash`, and then
   `source ~/CURI-Walker/walker_install/setup.bash`, and finally `catkin_make` in `ubt_sim_ws`.

3. Add `source ~/CURI-Walker/ubt_sim_ws/devel/setup.bash` into `~/.bashrc`. After that, `echo $ROS_PACKAGE_PATH`
   should give some output like this: `/home/x/CURI-Walker/ubt_sim_ws/src:/home/x/CURI-Walker/walker_install/share:/opt/ros/melodic/share`

# Execution

1. Run `roscore`

2. Start Webots, open `WAIC.wbt` in `walker_model/worlds/WAIC.wbt`

3. Refer README in walker_brain for usage guide.

4. Prepare to record the videos with [kazam].


## Launching MoveIt

To control the Walker robot via MoveIt you can use moveit.launch from the walker_webots_hardware_interface package.
This launcher will start the MoveIt move_group, together with the hardware interface to the simulated robot.

```
roslaunch walker_webots_hardware_interface moveit.launch show_rviz:=true
```

This configuration allows to control via MoveIt the arms, the head and the legs of the Walker robot. By default
the legs control is disabled to avoid interference with the gait module, you can enable it using enable_legs:=true

NOTE: for this to work you may need to install additional system packages, you can use rosdep to install them automatically.
Just move to ubt_sim_ws and run:

```
rosdep install --from-paths src --ignore-src -r -y
```

### Implementation details

Three different packages have been defined to handle the robot control.
* walker_description provides the urdf and the 3D meshes that represent the robot
* walker_moveit_config defines the MoveIt configuration
* walker_webots_hardware_interface provides the hardware_interface that connects the simulated robot to MoveIt.


# Cheat sheet

## Webots API

### Services

| **Name**      | **Type**                   | **Content**                                                                                                                      |
|---------------|----------------------------|----------------------------------------------------------------------------------------------------------------------------------|
| /walker/sence | webots\_api/SceneSelection | \.scene\_name “SwitchLight”, “GraspCup”, “PushCart”, “OpenFridge”, “CarryBox”, “Upstairs” \.nav true, false \.vision true, false |

#### Task-scene relation

| **scene name** | **nav/vision** | **task** |
|----------------|----------------|----------|
| SwitchLight    | F/F            | 1        |
| GraspCup       | F/F            | 2        |
|                | F/T            | 3        |
|                | T/F            | 4        |
|                | T/F            | 5        |
| PushCart       | F/F            | 6        |
|                | F/T            | 7        |
|                | T/F            | 8        |
|                | T/F            | 9        |
| OpenFridge     | F/F            | 10       |
|                | F/T            | 11       |
|                | T/F            | 12       |
|                | T/F            | 13       |
| CarryBox       | F/F            | 14       |
| Upstairs       | F/F            | 15       |

### Topics

| **Name**                        | **Type**                     | **Content**                                                                                                                                                                                                                                                                      |
|---------------------------------|------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| /Leg/DesiredJoint               | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
| /walker/Leg/MeasuredJoint       | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
|                                 |                              |                                                                                                                                                                                                                                                                                  |
| /sensor/head\_imu               | sensor\_msgs/Imu             |                                                                                                                                                                                                                                                                                  |
| /sensor/orientus\_imu           | sensor\_msgs/Imu             |                                                                                                                                                                                                                                                                                  |
| /sensor/camera\_imu             | sensor\_msgs/Imu             |                                                                                                                                                                                                                                                                                  |
|                                 |                              |                                                                                                                                                                                                                                                                                  |
| /sensor/ft/lankle               | geometry\_msgs/WrenchStamped |                                                                                                                                                                                                                                                                                  |
| /sensor/ft/lwrist               | geometry\_msgs/WrenchStamped |                                                                                                                                                                                                                                                                                  |
| /sensor/ft/rankle               | geometry\_msgs/WrenchStamped |                                                                                                                                                                                                                                                                                  |
| /sensor/ft/rwrist               | geometry\_msgs/WrenchStamped |                                                                                                                                                                                                                                                                                  |
|                                 |                              |                                                                                                                                                                                                                                                                                  |
| /walker/camera/bottomDepth      | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
| /walker/camera/bottomRGB        | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
| /walker/camera/doubleLeftRGB    | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
| /walker/camera/doubleRightRGB   | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
| /walker/camera/headDepth        | sensor\_msgs/Image           | +z: front, +x: right                                                                                                                                                                                                                             |
| /walker/camera/headRGB          | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
| /walker/camera/topDepth         | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
| /walker/camera/topRGB           | sensor\_msgs/Image           |                                                                                                                                                                                                                                                                                  |
|                                 |                              |                                                                                                                                                                                                                                                                                  |
| /walker/head/controller         | ubt\_core\_msgs/JointCommand | \.mode 5 \(P\), 6 \(V\), 7 \(F\) \.command\.resize\(2\) \[0\] HeadYaw, \[1\] HeadPitch                                                                                                                                                                                         |
| /walker/head/joint\_states      | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
| /walker/leftHand/controller     | ubt\_core\_msgs/JointCommand | \.mode 5 \(P\), 6 \(V\), 7 \(F\) \.command\.resize\(10\)  \[0\] LFirstFinger1, \[1\] LFirstFinger2, \[2\] LSecondFinger1, \[3\] LSecondFinger2,  \[4\] LThirdFinger1, \[5\] LThirdFinger2, \[6\] LForthFinger1, \[7\] LForthFinger2,  \[8\] LFifthFinger1, \[9\] LFifthFinger2 |
| /walker/leftHand/joint\_states  | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
| /walker/leftLimb/controller     | ubt\_core\_msgs/JointCommand | \.mode 5 \(P\), 6 \(V\), 7 \(F\) \.command\.resize\(7\)  \[0\] LShoulderPitch, \[1\] LShoulderRoll, \[2\] LShoulderYaw, \[3\] LElbowRoll,  \[4\] LElbowYaw, \[5\] LWristPitch, \[6\] LWristRoll                                                                                |
| /walker/leftLimb/joint\_states  | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
| /walker/rightHand/controller    | ubt\_core\_msgs/JointCommand | \.mode 5 \(P\), 6 \(V\), 7 \(F\) \.command\.resize\(10\)  \[0\] RFirstFinger1, \[1\] RFirstFinger2, \[2\] RSecondFinger1, \[3\] RSecondFinger2,  \[4\] RThirdFinger1, \[5\] RThirdFinger2, \[6\] RForthFinger1, \[7\] RForthFinger2,  \[8\] RFifthFinger1, \[9\] RFifthFinger2 |
| /walker/rightHand/joint\_states | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
| /walker/rightLimb/controller    | ubt\_core\_msgs/JointCommand | \.mode 5 \(P\), 6 \(V\), 7 \(F\) \.command\.resize\(7\)  \[0\] RShoulderPitch, \[1\] RShoulderRoll, \[2\] RShoulderYaw, \[3\] RElbowRoll,  \[4\] RElbowYaw, \[5\] RWristPitch, \[6\] RWristRoll                                                                                |
| /walker/rightLimb/joint\_states | sensor\_msgs/JointState      |                                                                                                                                                                                                                                                                                  |
|                                 |                              |                                                                                                                                                                                                                                                                                  |
| /walker/ultrasound/leftBack     | sensor\_msgs/Range           |                                                                                                                                                                                                                                                                                  |
| /walker/ultrasound/leftFront    | sensor\_msgs/Range           |                                                                                                                                                                                                                                                                                  |
| /walker/ultrasound/middleBack   | sensor\_msgs/Range           |                                                                                                                                                                                                                                                                                  |
| /walker/ultrasound/rightBack    | sensor\_msgs/Range           |                                                                                                                                                                                                                                                                                  |
| /walker/ultrasound/rightFront   | sensor\_msgs/Range           |                                                                                                                                                                                                                                                                                  |

## Gait API

### Services

| **Name**           | **Type**     | **Content**                                                   |
|--------------------|--------------|---------------------------------------------------------------|
| /Leg/TaskScheduler | walker\_srvs | \.func\_name “dynamic” \.param\_json “” \.cmd “start”, “stop” |

### Topics

| **Name**                                    | **Type**                    | **Content** |
|---------------------------------------------|-----------------------------|-------------|
| /ID\_status                                 | std\_msgs/String            |             |
| /Leg/BodyPlanner                            | geometry\_msgs/Point        |             |
| /Leg/CmdToDynamic                           | std\_msgs/Int64             |             |
| /Leg/DesiredJoint                           | sensor\_msgs/JointState     |             |
| /Leg/DisLeft2Arm                            | sensor\_msgs/JointState     |             |
| /Leg/DisRight2Arm                           | sensor\_msgs/JointState     |             |
| /Leg/FixedPlanning                          | geometry\_msgs/Pose2D       |             |
| /Leg/Joint2Sim                              | sensor\_msgs/JointState     |             |
| /Leg/RandomOutput                           | geometry\_msgs/PointStamped |             |
| /Leg/StepNum                                | std\_msgs/Int64             |             |
| /Leg/StopfromScript                         | std\_msgs/Bool              |             |
| /Leg/WaistPoseDesired                       | geometry\_msgs/PoseStamped  |             |
| /Leg/WaistPoseMeasured                      | geometry\_msgs/PoseStamped  |             |
| /Leg/dis\_waist2ankle\_left                 | geometry\_msgs/Point        |             |
| /Leg/dis\_waist2ankle\_right                | geometry\_msgs/Point        |             |
| /Leg/footpose2camera                        | geometry\_msgs/PoseStamped  |             |
| /Leg/footpose2waist                         | geometry\_msgs/PoseStamped  |             |
| /Leg/left\_leg\_ground                      | std\_msgs/Bool              |             |
| /Leg/leg\_status                            | std\_msgs/String            |             |
| /Leg/robot\_vel                             | geometry\_msgs/Pose2D       |             |
| /Leg/walking\_odom                          | nav\_msgs/Odometry          |             |
| /Leg/walking\_status                        | std\_msgs/String            |             |
| /Robot\_mode                                | geometry\_msgs/Twist        |             |
| /astra\_aruco/markers                       | aruco\_msgs/MarkerArray     |             |
| /astra\_aruco\_marker\_publisher\_2/markers | aruco\_msgs/MarkerArray     |             |
| /nav/cmd\_vel\_nav                          | geometry\_msgs/Twist        |             |
| /tf                                         | tf2\_msgs/TFMessage         |             |


# Issues

1. The Webots simulator could not run in real-time.

   According to the official statement, the dynamic simulation is time consuming where 0.2x real-time is an reasonable
   performance, so no need to worry about that and the finial videos will be judged basing on the simulation time.
   
2. Webots crash on startup.

   `/usr/local/bin/webots: line 86: 27518 Segmentation fault (core dumped) "$webots_home/bin/webots-bin" "$@"`
   
   Rollback any changes in the file `walker_model/worlds/.WAIC.wbproj`


[Source Code]: <https://github.com/DrawZeroPoint/CURI-Walker>
[ROS]: <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>
[kazam]: <https://linuxhint.com/record_screen_kazaam_ubuntu/>
