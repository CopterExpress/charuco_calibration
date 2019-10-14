# charuco_calibration
Camera calibration via charuco board package

## Installation
Create catkin workspace dir and sources dir in it (if you don't have catkin workspace):
```bash
mkdir -p ~/catkin_ws/src 
```

Execute catkin_make command:
```bash
catkin_make
```

Add environment variables that are needed for ROS:
```bash
source devel/setup.bash
```

You need to execute previous command on every bash login before starting node. If you want to do it automatically add this command to `/home/$USER/.bashrc` file.

## Running
Run calibration node for your computer by executing
```bash
roslaunch charuco_calibration calibration.launch
```

Run calibration node for remote computer on some `hostname` (e.g. on Raspberry Pi) by executing
```bash
ROS_MASTER_URI="http://hostname:11311" roslaunch charuco_calibration remote_calibration.launch
```

You will need special charuco board, which will be generated in `~/.ros/board.png` after first time of node executing. Print this board, measure square and marker lengths and enter these values as square_length and marker_length parameters of charuco_calibration_node in required `.launch` file. If you run the node first time, restart it after correcting `.launch` file.

Make about 20-25 different pictures by pressing "c" button on image window and then make camera calibration by pressing the "esc" button.

Calibration file will be saved in `~/.ros` directory as `calib.txt` file by default.

