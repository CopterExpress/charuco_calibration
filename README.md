# charuco_calibration

Camera calibration via charuco board ROS package.
> If you want to calibrate the camera on the embedded computer (like the Raspberry Pi), you should install this package on your laptop and use remote calibration launch file.

Currently the package provides the following nodes:

* `charuco_calibration_node` - an interactive ChArUco calibration node that uses an image topic to get calibration images;
* `calibrate_camera_from_files` - a non-interactive ChArUco calibration node that uses a list of files for calibration.

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

### `charuco_calibration_node`

Run calibration node for your computer by executing

```bash
roslaunch charuco_calibration calibration.launch
```

Run calibration node for remote computer on some `hostname` (e.g. on Raspberry Pi) by executing

```bash
ROS_MASTER_URI="http://hostname:11311" roslaunch charuco_calibration remote_calibration.launch
```

### `calibrate_camera_from_files`

Put all images that are to be used for calibration in a folder (for example, `/home/user/calibration_images`). After that, run:

```bash
roslaunch charuco_calibration files_calibration.launch images_path:=/home/user/calibration_images
```

> Do note that you need to specify the absolute path for calibration images!

The program will look for image files in the folder, select the ones that are suitable for calibration, and output a `calibration.yaml` file to the images folder.

> We strongly recommend using lossless image formats like `png` for source images.

## Parameters

### Common parameters

* `squares_x` - number of charuco board squares along x axis, default is 6.
* `squares_y` - number of charuco board squares along y axis, default is 8.
* `square_length` - length of square side in m, default is 0.021.
* `marker_length` - length of aruco side in m, default is 0.013.
* `dictionary_id` - id of aruco markers dictionary, default is 4 (DICT_5X5_50). The dictionary ID is as described in [OpenCV documentation](https://docs.opencv.org/3.4.9/d9/d6a/group__aruco.html#gac84398a9ed9dd01306592dd616c2c975) (DICT_4x4_50 is 0, DICT_4x4_100 is 1, etc).
* `aspect_ratio` - if `CALIB_FIX_ASPECT_RATIO` is set, use this aspect ratio for calibration.
* `perform_refinement` - attempt to find additional markers based on the currently detected ones (as described in [OpenCV documentation](https://docs.opencv.org/3.4.9/d9/d6a/group__aruco.html#ga13a2742381c0a48e146d230a8cda2e66)), default is false.
* `output_file` - name of the calibration file, default is `calibration.yaml`.

Parameters in `calibration_flags` namespace (see `launch/calibration_params.yaml`) are flags that are passed to `calibrateCameraCharuco`; their description can be found in [OpenCV documentation](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d).

Parameters in `detector_parameters` namespace (see `launch/calibration_params.yaml`) are used by ArUco detector and are described in [their own OpenCV documentation page](https://docs.opencv.org/3.4.9/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html). They can be tweaked to improve ArUco marker detection.

### `charuco_calibration_node` parameters

* `draw_historical_markers` - display points where key points for the board have been detected on the preview image, default is `true`.
* `save_images` - option to save all captured images to folder with calibration, default is `true`.

### `calibrate_camera_from_files` parameters

* `images_path` - path to the folder with images, default is `calibration_images`

> Note that the working directory for nodes launched from `roslaunch` is set to `~/.ros`. Use absolute paths whenever possible.

## Topics

`charuco_calibration_node` subscribes to `image` topic. Be sure to remap your actual image topic to it.

`charuco_calibration_node` publishes a `board` topic that contains the ChArUco board image.

`calibrate_camera_from_files` does not subscribe to any topic and does not publish any topics. It exits after the calibration is done or if it cannot be performed.

## Usage

### `charuco_calibration_node` usage

You will need special charuco board, which will be generated in `~/.ros/board.png` and published to the `board` topic after launching `charuco_calibration_node`. Print this board, measure square and marker lengths and enter these values as square_length and marker_length parameters of charuco_calibration_node in required `.launch` file. You can find directory with launch files with `roscd charuco_calibration/launch` command. If you run the node first time, restart it after you correct `.launch` file.

Make about 20-25 different pictures of the charuco board from different angles by pressing "c" key. Make camera calibration by pressing the "esc" key after you make enough frames. Blue calibration dots should cover the whole image during the process of calibration.

Calibration file will be saved in `~/.ros/calibration_<date>_<time>` directory as `calibration.yaml` file by default. You can change the name of the calibration file in the `output_file` parameter in the launch file.

Also, you can save all captured images to the calibration folder for further analysis if you specify `save_images` parameter as `true`.

The main goal of calibration is to get the calibration file with minimal reprojection error. Typical reprojection error is `0.3`, typical reprojection error for aruco markers is `1.0`.

#### Checking the calibration

You can check visual quality of calibration by viewing undistorted images from camera after the process of calibration.

Also, you can use [undistortion node](https://github.com/CopterExpress/clever_tools/blob/master/clever_tools/src/undistort_camera.py) from [clever_tools](https://github.com/CopterExpress/clever_tools) package to publish indistorted image to topic. You can view it using web_video_server or rqt_image_view ROS packages.

### `calibrate_camera_from_files` usage

If you already have source images for calibration (for example, after a previous run of `charuco_calibration_node`), you can run the non-interactive tool by running `roslaunch charuco_calibration files_calibration.launch images_path:=/path/to/calibration_images`. The calibration file will be placed at the specified path.

Note that all images at the folder are expected to have the same dimensions.

## Use calibration

Put the calibration file to the required directory for the program which requires it. For example, in the [clever](https://github.com/CopterExpress/clever) package the calibration file should be placed to the [camera_info](https://github.com/CopterExpress/clever/tree/master/clever/camera_info) folder and the path to the calibration file should be specified in the [main_camera.launch](https://github.com/CopterExpress/clever/blob/master/clever/launch/main_camera.launch#L22) file.
