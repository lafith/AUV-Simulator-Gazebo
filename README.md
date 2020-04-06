# AUV-SIMULATOR : TEAM TIBURON
This simulator, made by using Gazebo, is used to test the control algorithm, simulate underwater environment and transfer required sensor data for the AUV developed by **TEAM TIBURON- NIT ROURKELA**.

## Prerequisites
- [Ubuntu 18.04](https://ubuntu.com/download/desktop/thank-you?version=18.04.4&architecture=amd64#download-content).
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
-Gazebo9

## NOTE
This repository is still under development.

## INSTRUCTIONS
1. Set up ROS Melodic. Gazebo9 will be downloaded along with ROS.
2. ```mkdir -p ~/catkin_ws/src && cd catkin_ws && catkin build```
3. ```cd src```
3. Build [hammerhead](https://gitlab.com/tiburonnitr/hammerhead) package of Team Tiburon.
4. now clone this repository. ```git clone https://gitlab.com/makaara/makara_simulator.git```
5. ```sudo gedit ./bashrc``` and add following line at the end: ```export GAZEBO_MODEL_PATH=~/catkin_ws/src/sim_pkg/models/```
6. Restart the terminal and ```cd ~/catkin_ws && catkin build```
7.```source devel/setup.bash```
8. ```roslaunch sim_pkg sauvc_world.launch```

## IMPORTANT INFO:
- Currently buoyancy is simulated using [usv_gazebo_plugins](https://bitbucket.org/osrf/vrx/src/default/).
- **data_publisher** plugin send the imu information to the topic **/synchronizer/combined**.
-**thruster_plugin** receive PWM values from **/thruster_Speeds**. These are changed into thrust values using a [transformation function](https://github.com/aolgu003/RoboSub/blob/eda2261462f0aa5f5e1dbca1103f99c44f118f1f/Sea_Goat_2016/rov_sim/T100_thruster_dynamics.m).
- Images are continously sent to **/front_camera/image_rect_color** & **/bottom_camera/image_rect_color** using the gazebo [camera plugin](http://gazebosim.org/tutorials?tut=ros_gzplugins).

##TODO
- [ ] tweak the buoyancy plugin parameters and maybe mass of the auv, if needed, inorder to get better buoyancy effect.
- [ ] Simulate underwater environment effects like sun flares, blurry appearance, random image noise etc.
- [ ] Attempt curve fitting using [T100 thruster data (PWM vs Kgf)](https://github.com/aolgu003/RoboSub/blob/eda2261462f0aa5f5e1dbca1103f99c44f118f1f/Sea_Goat_2016/rov_sim/T100_motor_data.xlsx) to get better transform function.
