# BallonHand

BallonHand is a hybrid robotic gripper project centered on the design and validation of an integrated ROS-based controller for a tendon-actuated and pneumatically assisted hand.

As described in the thesis, the goal of the project is to make two actuation systems coexist and cooperate within a single control architecture:
- a tendon-driven mechanism responsible for finger closing and force grasping
- pneumatic soft pads that adapt the contact surface and support fine, compliant manipulation

This integration is meant to improve grasp effectiveness and move beyond simple open/close behavior, enabling more adaptive and dexterous object handling. The thesis also reports experimental validation of the system, including pressure control, selective actuator commands, and in-hand object rotation achieved through pneumatic actuation.

## Repository Contents

- `Cogripper/haria_cogripper`: main gripper firmware
- `Cogripper/haria_cogripper_dongle`: wireless dongle firmware
- `handball_ws/`: catkin workspace snapshot for the ROS control layer and trajectory files
- `handball_ws/src/pneumaticbox_traj_playback`: C++ ROS playback node and custom `goals.msg` message
- `handball_ws/src/pneumaticbox_traj_controller`: Python ROS node for pneumatic pressure handling
- `handball_ws/src/gripController`: Python ROS node that forwards grip commands over serial
- `firmware_motori/`: motor and pneumatic gripper control firmware
- `firmware_motori_edited/`: edited firmware variant
- `class_vanni_softpadcontroller.py`: soft pad control script
- `Tesi_Barro.pdf`: thesis documentation

The imported ROS workspace intentionally excludes generated `build/` and `devel/` artifacts. The vendored `rosserial` checkout from the original external workspace is also not mirrored here, so it should be installed separately if you need that dependency in a local ROS setup.

## Demo

![BallonHand demo](assets/ballonhand-demo.gif)
