# Fire fighting robot
By [Lidor Shimoni](https://github.com/lidorshimoni), [Daniel Angel](https://github.com/danielengel111)

* Uses SLAM to map its surrounding
* Autonomously exploring
* Recognizes fire and trapped peoples and marks their location on the map
<!-- ## Example
[![Watch the video](https://img.youtube.com/vi/CvRHvVOhvw0/default.jpg)](https://youtu.be/CvRHvVOhvw0) -->

## Example
![Image from simulation](else/simulation_image.png "Image from simulation")

![Costmap Visualization](else/rviz_costmap_image.png | width=100 "Costmap Visualization")

![simulation workspace](else/gazebo_image.png =250x250 "simulation workspace")


## Running in simulation
For a simulation with joystick controller use:


```
roslaunch joystick_slam.launch
```


For a simulation with move_base controller use:


```
roslaunch move_base_slam.launch
```

For autonomous exploration use:


```
roslaunch lite_exploration.launch
```


## Dependencies
The following dependencies are required:
* python 3.X
* python3-opencv
* teb_local_planner as move_base -> local_planner
* GlobalPlanner as move_base -> global_planner
* diffdrive
* frontier_exploration
### Optional
* Explore_lite
<!-- ## Refrences -->
