# ROS2 Packages for Scout Mobile Robot

## Packages

This repository contains minimal packages to control the scout robot using ROS. 

* scout_base: a ROS wrapper around [ugv_sdk](https://github.com/westonrobot/ugv_sdk) to monitor and control the scout robot
* scout_description: URDF model for the mobile base
* scout_msgs: scout related message definitions

## Supported Hardware

* Scout
* Scout Mini
* Scout Mini Omni

**Note:** Both V1 and V2 protocols are supported by this package (only with CAN interface). You don't have to specify the protocol version when launching the base node but you need to use the right launch file for your specific Scout model.  

## Basic usage of the ROS packages

1. Clone the packages into your colcon workspace and compile

    (the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ```
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/agilexrobotics/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/scout_ros2.git
    $ cd ..
    $ colcon build
    ```

2. Launch ROS nodes
 
* Start the base node for the Scout robot

    ```
    $ ros2 launch scout_base scout_base.launch.py
    ```

* Start the keyboard tele-op node

    ```
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## To run Scout2 on Gazebo 
Launch the launch file `display_scout.launch.py`

```
ros2 launch scout_description display_scout.launch.py
```


To run generic Keyboard Teleop, Run
```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

[scout_ros2_OS1.webm](https://github.com/arif09/scout_ros2/assets/5939058/4b19ce6f-120d-4e88-8862-61092535024f)

 

