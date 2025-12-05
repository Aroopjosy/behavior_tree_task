# ROS2 C++ Behavior Tree Node

This repository contains a ROS2 C++ node that executes robot tasks using BehaviorTree.CPP (behavior trees). It shows how to build a BT-based planner/agent

## Prerequisites 
* Ubuntu 22.04
* ROS 2 Humble
* colcon build tool
* BehaviorTree.CPP *no need to install*

## Workspace setup & Run

1. Create workspace dir
```
mkdir -p ros2_bt_ws/src
cd ros2_bt_ws/src
```
2. clone repo & build   
```
git clone https://github.com/Aroopjosy/behavior_tree_task.git .
cd ..
colcon build --symlink-install
```
3. source workspace & Run code
```
source ~/ros2_bt_ws/install/setup.bash
ros2 run apple_task_pkg bt_executor
```
Note: Output like 

```[INFO] [1764914509.744051979] [BT_Executor]: ACTION Executing: MoveToRoomDoor
[INFO] [1764914509.744061164] [BT_Executor]: CONDITION Checking: CheckDoorOpen
[INFO] [1764914509.744063974] [BT_Executor]: 
........
......