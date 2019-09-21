# Fire fighting robot
By [Lidor Shimoni](https://github.com/lidorshimoni), [Daniel Angel](https://github.com/danielengel111)

* Uses SLAM to map its surrounding
* Autonomously exploring
* Recognizes fire and trapped peoples and marks their location on the map
<!-- ## Example
[![Watch the video](https://img.youtube.com/vi/CvRHvVOhvw0/default.jpg)](https://youtu.be/CvRHvVOhvw0) -->

## Running
For a simulation with joystick controller use:


```
roslaunch joystick_slam.launch
```


For a simulation with move_base controller use:


```
roslaunch move_base_slam.launch
```

## Dependencies
The following python packges are required:
* python 3.*
* numpy**
* matplotlib**
* sklearn**
* sciPy**
* teb_local_planner for move_base
<!-- 
Particles Intersection (PI) is a method to fuse multiple particle filter estimators, where the cross-dependencies of the observations is unknown.
This package contain a PI implembtation for fusion the particle filters that estimates the state of a robot in a flat space.
This package contain two main nodes:
* particle_filter.py - an implemantation of a particle filter for robot localization using [1].
* particlesintersection.py - an implemantation of a particle intersection for cooperative localization using [2].


  -->
