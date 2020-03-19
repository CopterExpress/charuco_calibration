/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include "calibrator.h"
#include "utils.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdlib.h>
#include <iomanip>
#include <sstream>

#include <sys/stat.h>
#include <sys/types.h>

#include <vector>
#include <iostream>
#include <ctime>
#include <fstream>

using namespace cv;

cv::Mat lastImage;
bool hasImage = false;

void imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img);
    lastImage = cv_img->image;
    hasImage = true;
}

static void logFunction(charuco_calibration::LogLevel logLevel, const std::string& message)
{
    ros::console::levels::Level rosLogLevel;
    switch(logLevel)
    {
        case charuco_calibration::LogLevel::DEBUG:
            rosLogLevel = ros::console::levels::Level::Debug;
            break;
        case charuco_calibration::LogLevel::INFO:
            rosLogLevel = ros::console::levels::Level::Info;
            break;
        case charuco_calibration::LogLevel::WARN:
            rosLogLevel = ros::console::levels::Level::Warn;
            break;
        case charuco_calibration::LogLevel::ERROR:
            rosLogLevel = ros::console::levels::Level::Error;
            break;
        case charuco_calibration::LogLevel::FATAL:
            rosLogLevel = ros::console::levels::Level::Fatal;
            break;
        default:
            // This should never happen, but if it does, everything went south
            rosLogLevel = ros::console::levels::Level::Fatal;
    }
    ROS_LOG(rosLogLevel, ROSCONSOLE_DEFAULT_NAME, "%s", message.c_str());
}

/**
 */
int main(int argc, char *argv[]) {
    using namespace charuco_calibration;
    ros::init(argc, argv, "cv_calib");

    std::string imgPath;

    // Get current datetime
    auto t = time(nullptr);
    auto tm = *localtime(&t);
    std::ostringstream oss;
    oss << "calibration_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    auto datetime = oss.str();

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    charuco_calibration::Calibrator calibrator;
    readCalibratorParams(nhPriv, calibrator);

    bool saveCalibrationImages = nhPriv.param<bool>("save_images", true);
    std::string outputFile = nhPriv.param<std::string>("output_file", "calibration.yaml");
    
    // Make folder with timedate name
    mkdir(datetime.c_str(), 0775);

    // Get output filepath
    oss << "/" << outputFile;
    auto outputFilePath = oss.str();

    ros::NodeHandle nh_detector("~detector_parameters");
    readDetectorParameters(nh_detector, calibrator.arucoDetectorParams);

    //image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_transport::ImageTransport it(nh);
    image_transport::ImageTransport itPriv(nhPriv);

    int waitTime = 10;
    ROS_INFO("Subscribing to image topic");
    auto sub = it.subscribe("image", 1, imageCallback /*, hints */);
    ROS_INFO("Advertising charuco board image");
    auto pub = nhPriv.advertise<sensor_msgs::Image>("board", 1, true);

    int boardImgWidth = nhPriv.param("board_image_width", 1536);
    int boardImgHeight = nhPriv.param("board_image_height", 2048);
    int boardImgBorder = nhPriv.param("board_image_border", 100);

    auto boardImg = calibrator.getBoardImage(boardImgWidth, boardImgHeight, boardImgBorder);

    cv::imwrite("board.png", boardImg);
    cv_bridge::CvImage boardImgBridge;
    boardImgBridge.image = boardImg;
    boardImgBridge.encoding = sensor_msgs::image_encodings::MONO8;
    pub.publish(boardImgBridge.toImageMsg());

    while(!hasImage)
    {
        ros::spinOnce();
    }

    int imgCounter = 1;

    calibrator.setLogger(logFunction);

    while(hasImage) {
        Mat image;
        image = lastImage.clone();

        auto detectionResult = calibrator.processImage(image);
        auto displayedImage = calibrator.drawDetectionResults(detectionResult);

        putText(displayedImage, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imshow("out", displayedImage);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
        if(key == 'c') {
            if (detectionResult.isValid()) {
                std::cout << "Frame " << imgCounter << " captured and saved" << std::endl;
                calibrator.addToCalibrationList(detectionResult);
                if (saveCalibrationImages) {
                    imgPath = datetime + "/" + std::to_string(imgCounter) + ".png";
                    imwrite(imgPath.c_str(), image);
                }
                imgCounter++;
            }
            else {
                std::cout << "Frame rejected" << std::endl;
            }
        }
        ros::spinOnce();
        if (ros::isShuttingDown())
        {
            return 0;
        }
    }

    cvDestroyWindow("out");

    std::cout << "Calibrating..." << std::endl;

    auto calibResult = calibrator.performCalibration();

    if (calibResult.isValid)
    {
        std::ofstream outFile(outputFilePath);
        outFile << "# File generated by charuco_calibration" << std::endl;
        saveCameraInfo(outFile, calibResult);
        if (!outFile)
        {
            std::cerr << "Encountered an error while writing result" << std::endl;
        }
        else
        {
            std::cout << "Calibration saved to " << outputFilePath << std::endl;
            std::cout << "Check undistorted images from camera. Press esc to exit." << std::endl;
        }
    }

    while(hasImage) {
        Mat image, imageUndistorted;
        image = lastImage.clone();
        imageUndistorted = image.clone();

        undistort(image, imageUndistorted, calibResult.cameraMatrix, calibResult.distCoeffs);

        imshow("Undistorted Sample", imageUndistorted);
        char key = (char)waitKey(waitTime);
        if(key == 27) ros::shutdown();
        ros::spinOnce();
        if (ros::isShuttingDown())
        {
            return 0;
        }
    }

    return 0;
}
