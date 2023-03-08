# robotiq_bringup



## Usage

### Usage with `forward_command_controller/ForwardCommandControllerr`

```
ros2 launch robotiq_bringup robotiq_2f_85_system_position_only.launch.py robot_controller:=robotiq_gripper_controller
```
### Send commands via `robotiq_gripper_controller`
```
ros2 topic pub /robotiq_gripper_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5"
```


### Usage with `position_controllers/GripperActionController`

```
ros2 launch robotiq_bringup robotiq_2f_85_system_position_only.launch.py robot_controller:=robotiq_gripper_action_controller
```
#### Send commands via `robotiq_gripper_action_controller`

```

ros2 action send_goal /robotiq_gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.5, max_effort: 10.0}}"
```
