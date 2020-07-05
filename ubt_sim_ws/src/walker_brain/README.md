# Walker Brain

Main entrance for solving the tasks of WAIC 2020 Walker challenge.

# Pre-requests

```
ros-melodic-behaviortree-cpp-v3
```

# Usage

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
for the loading of the MoveIt planning interface (when the `You can start planning now` message shows up).

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

