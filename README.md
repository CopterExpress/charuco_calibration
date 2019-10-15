# charuco_calibration
Camera calibration via charuco board ROS package.
> If you want to calibrate the camera on the embedded computer (like the Raspberry Pi), you should install this package on your laptop and use remote calibration launch file. 

## Installation
Create catkin workspace dir and sources dir in it (if you don't have catkin workspace):
```bash
mkdir -p ~/catkin_ws/src 
```

Clone this repo to `~/catkin_ws/src` directory:
```bash
git clone https://github.com/CopterExpress/charuco_calibration.git 
```

Execute catkin_make command from catkin workspace directory:
```bash
cd ~/catkin_ws
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

You will need special charuco board, which will be generated in `~/.ros/board.png` after first time of node executing. Print this board, measure square and marker lengths and enter these values as square_length and marker_length parameters of charuco_calibration_node in required `.launch` file. You can find directory with launch files with `roscd charuco_calibration/launch` command. If you run the node first time, restart it after you correct `.launch` file.

Make about 20-25 different pictures by pressing "c" button on image window and then make camera calibration by pressing the "esc" button. Blue calibration dots should cover the whole image.

Calibration file will be saved in `~/.ros` directory as `calibration.yaml` file by default. You can change path to file and filename in the `filepath` parameter in the launch file. Specify the path to this file to the program, which require it.

The main goal of calibration - get the calibration file with minimal reprojection error. Typical reprojection error is `0.5`, typical reprojection error for aruco markers is `1.0`.

## Check calibration
You can use [undistortion node](https://github.com/CopterExpress/clever_tools/blob/master/clever_tools/src/undistort_camera.py) from [clever_tools](https://github.com/CopterExpress/clever_tools) package to publish indistorted image to topic. You can view it using web_video_server or rqt_image_view ROS packages.



