# Linear Model Predictive Control for Quadrotor

## Setup

---

```bash
$ cd <catkin workspace>/src
$ git clone git@github.com:Masa0u0/multirotor_controller.git
$ cd ..
$ vcs import src < src/multirotor_controller/.rosinstall
$ rosdep install --from-paths src --ignore-src -ry
```

## Execute

---

```bash
$ roslaunch rotors_gazebo mav.launch mav_name:=iris
$ roslaunch multirotor_controller bringup.launch
```
