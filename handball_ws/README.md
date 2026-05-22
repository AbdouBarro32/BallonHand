# handball_ws

This folder contains the BallonHand ROS workspace sources that were missing from the initial repository import.

Included here:
- `src/pneumaticbox_traj_playback`: trajectory playback node and custom message definition
- `src/pneumaticbox_traj_controller`: pneumatic pressure handling node
- `src/gripController`: serial bridge for grip commands
- `prova.txt`, `prova1.txt`, `rotazione.txt`: example command and manipulation trajectories

Not included here:
- generated catkin outputs such as `build/` and `devel/`
- the separate vendored `rosserial` checkout from the original external workspace
