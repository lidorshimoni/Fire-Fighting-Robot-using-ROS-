# Fire fighting robot
By [Lidor Shimoni](https://github.com/lidorshimoni), [Daniel Angel](https://github.com/danielengel111)

* Uses SLAM to map its surrounding
* Autonomously exploring
* Recognizes fire and trapped peoples and marks their location on the map
<!-- ## Example
[![Watch the video](https://img.youtube.com/vi/CvRHvVOhvw0/default.jpg)](https://youtu.be/CvRHvVOhvw0) -->

## Running in simulation
For a simulation with joystick controller use:


```
roslaunch joystick_slam.launch
```


For a simulation with move_base controller use:


```
roslaunch move_base_slam.launch
```

## Dependencies
The following dependencies are required:
* python 3.X
* python3-opencv
* teb_local_planner as move_base -> local_planner
* NavfnROS as move_base -> global_planner
* diffdrive

<!-- ## Refrences -->
