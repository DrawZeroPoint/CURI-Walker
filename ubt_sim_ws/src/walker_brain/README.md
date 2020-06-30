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

**Note that for control or vision only tasks the launch file will automatically call this
service for you**


## Run the solutions

The common use case for executing a given task in the contest involves 2 steps:

First, run the **preparation** launch file like:

```
roslaunch walker_brain prepare_switch_light.launch nav:=false
```

Then, run the **execution** launch file like:

```
roslaunch walker_brain switch_light.launch
```

**It is suggested to wait for about 10 seconds after launching the prepare file and before launching
the execution file.**

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

Particularly, for the grasp_cup task, you need also specify the id of the cup
to grasp:

```
roslaunch walker_brain grasp_cup.launch task_id:=3 cup_id:=1
```

