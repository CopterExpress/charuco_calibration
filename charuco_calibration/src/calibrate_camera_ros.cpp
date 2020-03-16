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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
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

/**
 */
static bool readDetectorParameters(ros::NodeHandle& nh, Ptr<aruco::DetectorParameters> &params) {
#define GET_PARAM(paramName) {params->paramName = nh.param(#paramName, params->paramName); ROS_INFO_STREAM(#paramName " set to " << params->paramName);}
    GET_PARAM(adaptiveThreshWinSizeMin);
    GET_PARAM(adaptiveThreshWinSizeMax);
    GET_PARAM(adaptiveThreshWinSizeStep);
    GET_PARAM(adaptiveThreshConstant);
    GET_PARAM(minMarkerPerimeterRate);
    GET_PARAM(maxMarkerPerimeterRate);
    GET_PARAM(polygonalApproxAccuracyRate);
    GET_PARAM(minCornerDistanceRate);
    GET_PARAM(minDistanceToBorder);
    GET_PARAM(minMarkerDistanceRate);
#if (CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 3)
    GET_PARAM(cornerRefinementMethod);
#else
    // Older OpenCV versions only have doCornerRefinement
    int cornerRefinementMethod = nh.param("cornerRefinementMethod", 0);
    if (cornerRefinementMethod > 1)
    {
        ROS_WARN_STREAM("cornerRefinementMethod set to " << cornerRefinementMethod << ", but current OpenCV version only supports subpixel refinement");
    }
    params->doCornerRefinement = bool(cornerRefinementMethod);
#endif
    GET_PARAM(cornerRefinementWinSize);
    GET_PARAM(cornerRefinementMaxIterations);
    GET_PARAM(cornerRefinementMinAccuracy);
    GET_PARAM(markerBorderBits);
    GET_PARAM(perspectiveRemovePixelPerCell);
    GET_PARAM(perspectiveRemoveIgnoredMarginPerCell);
    GET_PARAM(maxErroneousBitsInBorderRate);
    GET_PARAM(minOtsuStdDev);
    GET_PARAM(errorCorrectionRate);
#undef GET_PARAM
    return true;
}

static bool readCalibrationFlags(ros::NodeHandle& nh, int& calibrationFlags)
{
    if (!nh.hasParam("calibration_flags"))
    {
        return false;
    }
    calibrationFlags = 0;
#define GET_FLAG(flag_name) { bool flagSet = nh.param("calibration_flags/" #flag_name, false); calibrationFlags |= flagSet ? cv::flag_name : 0; ROS_INFO_STREAM("Calibration flag " #flag_name " set to " << flagSet);}
    GET_FLAG(CALIB_USE_INTRINSIC_GUESS);
    GET_FLAG(CALIB_FIX_ASPECT_RATIO);
    GET_FLAG(CALIB_FIX_PRINCIPAL_POINT);
    GET_FLAG(CALIB_ZERO_TANGENT_DIST);
    GET_FLAG(CALIB_FIX_FOCAL_LENGTH);
    GET_FLAG(CALIB_FIX_K1);
    GET_FLAG(CALIB_FIX_K2);
    GET_FLAG(CALIB_FIX_K3);
    GET_FLAG(CALIB_FIX_K4);
    GET_FLAG(CALIB_FIX_K5);
    GET_FLAG(CALIB_FIX_K6);
    GET_FLAG(CALIB_RATIONAL_MODEL);
    GET_FLAG(CALIB_THIN_PRISM_MODEL);
    GET_FLAG(CALIB_FIX_S1_S2_S3_S4);
    GET_FLAG(CALIB_TILTED_MODEL);
    GET_FLAG(CALIB_FIX_TAUX_TAUY);
    GET_FLAG(CALIB_USE_QR);
    GET_FLAG(CALIB_FIX_INTRINSIC);
    GET_FLAG(CALIB_SAME_FOCAL_LENGTH);
    GET_FLAG(CALIB_ZERO_DISPARITY);
    GET_FLAG(CALIB_USE_LU);
#if (CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 3)
    GET_FLAG(CALIB_FIX_TANGENT_DIST);
    GET_FLAG(CALIB_USE_EXTRINSIC_GUESS);
#endif
#undef GET_FLAG
    return true;
}

/**
 * Get ROS-compatible distortion model from calibration result.
 * 
 * @param calibration Calibration result, as returned by Calibrator::performCalibration
 * @return An std::string containing the name of the distortion model as described by REP-104
 */
static std::string distortionModelFromResult(const charuco_calibration::CalibrationResult& calibration)
{
    // Do the dumb thing for now: count the number of distortion coefficients
    int numCoeffs = calibration.distCoeffs.size().width;
    switch (numCoeffs)
    {
        // 4 coefficients: probably a fisheye camera?
        case 4:
            return sensor_msgs::distortion_models::EQUIDISTANT;
        // 5 coefficients: simple, 5-parameter polynomial ("Plumb Bob", https://www.ros.org/reps/rep-0104.html#alternate-distortion-models)
        case 5:
            return sensor_msgs::distortion_models::PLUMB_BOB;
        // 8 coefficients: rational polynomial
        // 12 coefficients: rational polynomial + thin prism distortion
        // 14 coefficients: rational polynomial + thin prism + sensor tilt
        case 8:
        case 12:
        case 14:
            return sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
        // Other sizes should not be possible, but we'll default to the "Plumb Bob" with a warning
        default:
            return sensor_msgs::distortion_models::PLUMB_BOB + " # Possibly incorrect";
    }
}

/**
 * Output cv::Mat as a yaml record to an output stream.
 * 
 * @param output A reference to a derivative of std::ostream (a file stream, for example).
 * @param matName Name of the output matrix (as should be written in the yaml file).
 * @param mat A cv::Mat containing the data
 * @return Reference to the output stream
 * @note This is a hacky way to output matrix data; it does not support outputting to a nested YAML element,
 *       and only supports CV_64F matrices. Still, it's enough for our case.
 */
static std::ostream& cvToYamlArray(std::ostream& output, const std::string& matName, const cv::Mat& mat)
{
    output << matName << ":" << std::endl;
    output << "  rows: " << mat.size().height << std::endl;
    output << "  cols: " << mat.size().width << std::endl;
    output << "  data: [ ";
    for(const auto& value : cv::Mat_<double>(mat))
    {
        output << value << ", ";
    }
    output << "]" << std::endl;
    return output;
}

/**
 * Output camera parameters to an output stream, using ROS-compatible YAML syntax
 * 
 * @param result Calibration result returned by the Calibrator object.
 * @param output A derivative of the std::ostream class where you wish to put the data.
 * @return The (possibly changed) output stream.
 */
static std::ostream& saveCameraInfo(std::ostream& output, charuco_calibration::CalibrationResult& result)
{
    // Field order does not matter here, but still
    output << "image_width: " << result.imgSize.width << std::endl;
    output << "image_height: " << result.imgSize.height << std::endl;
    output << "distortion_model: " << distortionModelFromResult(result) << std::endl;
    // FIXME: Grab camera name from somewhere?
    output << "camera_name: " << "camera" << std::endl;
    cvToYamlArray(output, "camera_matrix", result.cameraMatrix);
    cvToYamlArray(output, "distortion_coefficients", result.distCoeffs);
    // FIXME: Add recrification matrix to calibration results?
    cvToYamlArray(output, "rectification_matrix", cv::Mat::eye(3, 3, CV_64F));
    // FIXME: Add projection matrix to calibration results?
    cv::Mat projectionMatrix = cv::Mat::zeros(3, 4, CV_64F);
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            projectionMatrix.at< double >(i, j) = result.cameraMatrix.at<double>(i, j);
        }
    }
    cvToYamlArray(output, "projection_matrix", projectionMatrix);
    return output;
}

static void readCalibratorParams(ros::NodeHandle& nh, charuco_calibration::Calibrator& calibrator)
{
    calibrator.params.squaresX = nh.param("squares_x", 6);
    calibrator.params.squaresY = nh.param("squares_y", 8);
    calibrator.params.squareLength = nh.param("square_length", 0.021);
    calibrator.params.markerLength = nh.param("marker_length", 0.013);
    calibrator.params.dictionaryId = nh.param("dictionary_id", 4);
    calibrator.params.aspectRatio = nh.param("aspect_ratio", 1.0);
    calibrator.params.performRefinement = nh.param("perform_refinement", false);
    calibrator.params.drawHistoricalMarkers = nh.param("draw_historical_markers", true);
    // FIXME: Make flags usage more user-friendly
    if (!readCalibrationFlags(nh, calibrator.params.calibrationFlags))
    {
        ROS_WARN("Could not retrieve calibration_flags from parameter server");
        calibrator.params.calibrationFlags = nh.param<int>("calibration_flags_mask", cv::CALIB_RATIONAL_MODEL);
    }
    else
    {
        ROS_INFO_STREAM("Calibration flags set to " << calibrator.params.calibrationFlags);
    }
    if (std::abs(calibrator.params.aspectRatio - 1.0) > 0.01)
    {
        if (!(calibrator.params.calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO))
        {
            ROS_WARN_STREAM("Aspect ratio is " << calibrator.params.aspectRatio << ", but CALIB_FIX_ASPECT_RATIO is not set");
        }
    }
    calibrator.applyParams();
}

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

    bool showChessboardCorners = true;

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

    auto boardImg = calibrator.getBoardImage(2048, 1536, 100);

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
