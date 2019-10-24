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

Parameters:
* `squares_x` - number of charuco board squares along x axis, default is 6
* `squares_y` - number of charuco board squares along y axis, default is 8
* `square_length` - length of square side in m
* `marker_length` - length of aruco side in m
* `dictionary_id` - id of aruco markers dictionary, default is 4 (DICT_5X5_50). You can view all dictionary id's descriptions at the top of the [source code](charuco_calibration/src/calibrate_camera_ros.cpp)
* `save_images` - option to save all captured images to folder with calibration, default is `true`
* `output_file` - name of the calibration file, default is `calibration.yaml`

You will need special charuco board, which will be generated in `~/.ros/board.png` after first time of node executing. Print this board, measure square and marker lengths and enter these values as square_length and marker_length parameters of charuco_calibration_node in required `.launch` file. You can find directory with launch files with `roscd charuco_calibration/launch` command. If you run the node first time, restart it after you correct `.launch` file.

Make about 20-25 different pictures of the charuco board from different angles by pressing "c" key. Make camera calibration by pressing the "esc" key after you make enough frames. Blue calibration dots should cover the whole image during the process of calibration.

Calibration file will be saved in `~/.ros/calibration_<date>_<time>` directory as `calibration.yaml` file by default. You can change path to output file in the `output_file` parameter in the launch file. Specify the path to this file to the program, which require it.

Also, you can save all captured images to this folder for analysis, if you specify `save_images` parameter as `true`.

The main goal of calibration - get the calibration file with minimal reprojection error. Typical reprojection error is `0.3`, typical reprojection error for aruco markers is `1.0`.

## Check calibration

You can check visual quality of calibration by viewing undistorted images from camera after the process of calibration.

Also, you can use [undistortion node](https://github.com/CopterExpress/clever_tools/blob/master/clever_tools/src/undistort_camera.py) from [clever_tools](https://github.com/CopterExpress/clever_tools) package to publish indistorted image to topic. You can view it using web_video_server or rqt_image_view ROS packages.



