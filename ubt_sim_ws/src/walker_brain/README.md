# Walker Brain

Main entrance for solving the tasks of WAIC 2020 Walker challenge.

# Pre-requests

This package used BehaviorTree.CPP and corresponding ROS package that could be not available in
classical ROS install, all you need is to install the followings by `sudo apt-get`

```
ros-melodic-behaviortree-cpp-v3
```

# Usage Guide

## Short version

Hereby we list the commands you need to use for executing each task. The common preparation for all 
these tasks is to start `roscore` and Webots, loading the official world model and start the simulation.
For a detailed guide, please refer to the following parts.

| Task id | Tasks       | Step 1                                                                     | Initialization call \(none means automatically called\)           | Step 2                                                    | Step 3                                                             | Note                                                    |
|---------|-------------|----------------------------------------------------------------------------|-----------------------------------------------------------------------------------|-----------------------------------------------------------|--------------------------------------------------------------------|---------------------------------------------------------|
| 1       | SwitchLight | roslaunch walker\_brain prepare\_switch\_light\.launch                     | none                                                                              | Wait for the preparation finish                           | roslaunch walker\_brain switch\_light\.launch                      |                                                         |
| 2       | GraspCup    | roslaunch walker\_brain prepare\_grasp\_cup\.launch nav:=false             | rosservice call /walker/sence "scene\_name: 'GraspCup' nav: false vision: false"  | Wait for the preparation finish                           | roslaunch walker\_brain grasp\_cup\.launch task\_id:=2 cup\_id:=ID | ID is the cup number obtained after initialization call |
| 3       | GraspCup    | roslaunch walker\_brain prepare\_grasp\_cup\.launch nav:=false             | rosservice call /walker/sence "scene\_name: 'GraspCup' nav: false vision: true"   | Wait for the preparation finish                           | roslaunch walker\_brain grasp\_cup\.launch task\_id:=3 cup\_id:=ID | ID is the cup number obtained after initialization call |
| 4       | GraspCup    | roslaunch walker\_brain prepare\_grasp\_cup\.launch nav:=true              | rosservice call /walker/sence "scene\_name: 'GraspCup' nav: true vision: false"   | Initialize the robot pose in RViz with pose estimate tool | roslaunch walker\_brain grasp\_cup\.launch task\_id:=4 cup\_id:=ID | ID is the cup number obtained after initialization call |
| 5       | GraspCup    | roslaunch walker\_brain prepare\_grasp\_cup\.launch nav:=true              | rosservice call /walker/sence "scene\_name: 'GraspCup' nav: true vision: false"   | Initialize the robot pose in RViz with pose estimate tool | roslaunch walker\_brain grasp\_cup\.launch task\_id:=5             |                                                         |
| 6       | PushCart    | roslaunch walker\_brain prepare\_push\_cart\.launch nav:=false             | none                                                                              | Wait for the preparation finish                           | roslaunch walker\_brain push\_cart\.launch task\_id:=6             |                                                         |
| 7       | PushCart    | roslaunch walker\_brain prepare\_push\_cart\.launch nav:=false             | none                                                                              | Wait for the preparation finish                           | roslaunch walker\_brain push\_cart\.launch task\_id:=7             |                                                         |
| 8       | PushCart    | roslaunch walker\_brain prepare\_push\_cart\.launch nav:=true              | rosservice call /walker/sence "scene\_name: 'PushCart' nav: true vision: false"   | Initialize the robot pose in RViz with pose estimate tool | roslaunch walker\_brain push\_cart\.launch task\_id:=8             |                                                         |
| 9       | PushCart    | roslaunch walker\_brain prepare\_push\_cart\.launch nav:=true              | rosservice call /walker/sence "scene\_name: 'PushCart' nav: true vision: false"   | Initialize the robot pose in RViz with pose estimate tool | roslaunch walker\_brain push\_cart\.launch task\_id:=9             |                                                         |
| 10      | OpenFridge  | roslaunch walker\_brain prepare\_open\_fridge\.launch nav:=false           | none                                                                              | Wait for the preparation finish                           | roslaunch walker\_brain open\_fridge\.launch task\_id:=10          |                                                         |
| 11      | OpenFridge  | roslaunch walker\_brain prepare\_open\_fridge\.launch nav:=false           | none                                                                              | Wait for the preparation finish                           | roslaunch walker\_brain open\_fridge\.launch task\_id:=11          |                                                         |
| 12      | OpenFridge  | roslaunch walker\_brain prepare\_open\_fridge\.launch nav:=true            | rosservice call /walker/sence "scene\_name: 'OpenFridge' nav: true vision: false" | Initialize the robot pose in RViz with pose estimate tool | roslaunch walker\_brain open\_fridge\.launch task\_id:=12          |                                                         |
| 13      | OpenFridge  | roslaunch walker\_brain prepare\_open\_fridge\.launch nav:=true            | rosservice call /walker/sence "scene\_name: 'OpenFridge' nav: true vision: false" | Initialize the robot pose in RViz with pose estimate tool | roslaunch walker\_brain open\_fridge\.launch task\_id:=13          |                                                         |
| 14      | CarryBox    | To hurry to implement in walker\_brain, refer walker\_movement instead :\) |                                                                                   |                                                           |                                                                    |                                                         |
| 15      | Upstairs    | roslaunch walker\_brain prepare\_upstairs\.launch                          | none                                                                              | Wait for the preparation finish                           | roslaunch walker\_brain upstairs\.launch                           |                                                         |


## Before running walker brain

Start the Webots simulation and initialize the task setting by

```
rosservice call /walker/sence ..(Tab to complete)
```

**Note**: For task SwitchLight, OpenFridge (control or vision only mode), 
PushCart (control or vision only mode), the launch file will **automatically** 
call this service for you. However, you need to manually 
call the service for all the navigation tasks, besides all GraspCup tasks, 
since the cup id needs initialization. 


## Run the solutions

The common use case for executing a given task in the contest involves 2 steps:

First, run the **preparation** launch file like:

```
roslaunch walker_brain prepare_switch_light.launch
```

Then, run the **execution** launch file like:

```
roslaunch walker_brain switch_light.launch
```

**Note**: It is mandatory to wait for about 10 seconds after launching the prepare file, giving some time 
for the loading of the MoveIt planning interface (at least you should see the `You can start planning now` message shows up).

## Launching parameters

The preparation launch file of some tasks have a `nav` param that must be given,
which could be `true` or `false`. Set it to be true if the task runs in navigation
or navigation+ mode:

```
roslaunch walker_brain prepare_push_cart.launch nav:=true
```

Note that some tasks (i.e., grasp_cup, push_cart, open_fridge) have some 
variants that could be distinguished by the `task_id` parameter, 
you should specify that like:

```
roslaunch walker_brain push_cart.launch task_id:=7
```

or otherwise errors would be raised.

Particularly, for the grasp_cup task, you need also specify the id (in range [1, 5]) of the cup
to grasp if you are performing task 2, 3, or 4. For task 5, cup_id takes no effect:

```
roslaunch walker_brain grasp_cup.launch task_id:=3 cup_id:=1
```

