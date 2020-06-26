# Walker Brain

Main entrance for solving the tasks of WAIC 2020 Walker challenge.

# Usage

## Before running walker brain

You should first start the Webots simulation and initialize the task setting by

```
rosservice call /walker/sence ..(Tab to complete)
``` 

## Running the solutions

The common use case for executing a given task in the contest involves 2 steps:

First, run the **preparation** launch file like:

```
roslaunch walker_brain prepare_switch_light.launch
```

Then, run the **execution** launch file like:

```
roslaunch walker_brain switch_light.launch
```

Note that some tasks (i.e., grasp_cup, push_cart, open_fridge) have some 
variants that could be distinguished by the `task_id` parameter, 
you should specify that like:

```
roslaunch walker_brain push_cart.launch task_id:=7
```

or otherwise the default task id would be used.

Particularly, for the grasp_cup task, you need also specify the id of the cup
to grasp:

```
roslaunch walker_brain grasp_cup.launch task_id:=3 cup_id:=1
```

