# evotrainer_drive
A ROS2 port of the [studierbot](https://github.com/autonohm/robotworkshop/tree/motorControllerRedesignhttps://github.com/autonohm/robotworkshop/tree/motorControllerRedesign) control package for legacy [EduArt](https://github.com/EduArt-Robotik) / [Evocortex](https://github.com/evocortex) motor controllers

# Quick Start
```bash
#check that CAN network is present
ip a
#if its down use:
sudo ip link set can0 up type can bitrate 500000

#build the node as usual, and launch with
ros2 launch evotrainer_drive evotrainer_drive.launch.py
```
