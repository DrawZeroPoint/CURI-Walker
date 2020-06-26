# Walker navigation package

## Instalation
sudo apt install  ros-melodic-eband-local-planner ros-melodic-pointcloud-to-laserscan ros-melodic-depthimage-to-laserscan ros-melodic-gmapping ros-melodic-slam-gmapping ros-melodic-rtabmap-ros ros-melodic-rtabmap


## Execution

The order of the processes i used to execute the navigation tasks were, where each has its own terminal, exept the service calls.

1. roscore
2. webots walker_model/worlds/WAIC.wbtb
3. Ros service call for the scene [1] after webots loads.
4. rviz (A config is provided in configs folder)
5. roslaunch walker_nav gmapping_test.launch
6. Inside Rviz, after having the map and local costmap loaded, provide a 2d estimate pose, using rviz tool. It does not need precision, it needs to be roughly in the correct position and orientation, see navigation task videos.
7. ros service call for the walking start [2]



[1]
```bash 
rosservice call /walker/sence "scene_name: 'OpenFridge' 
nav: true
vision: false"
```

[2]
```bash 
rosservice call /Leg/TaskScheduler "func_name: 'dynamic'
param_json: ''
cmd: 'start'" 
``` 

## Summary of gmapping launch file

1. Launch leg_motion walker2_leg.launch (with user_account.json on the walker_nav package)
2. camera_info.py script created that subscribes to multiple data streams and republishes them [3]
3. Nodelet manager and nodelet for depth image to point cloud conversion
4. rtabmap visual odometry
5. Carlo's webots moveit! package (mainly for TFs)
6. nodelet for depthimage_to_laserscan (Can be removed, is not used)
7. nodelet for pointlcoud_to_laserscan
8. walker movebase configuration
9. map server publishing the map in the map folder
10. amcl initialization


[3]
1. Head IMU
2. Head rgbd + camera infos
3. bottom rgbd + camera infos
4. right and left fisheye cameras + camera info


## Calling the movement service

The mov.py file in the scripts folder presents an example of the service call and, at the same time, is the code used to execute the navigation tasks, including the poses.