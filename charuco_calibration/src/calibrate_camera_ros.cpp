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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <dirent.h>

#include <vector>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

namespace {
const char* about =
        "Calibration using a ChArUco board\n"
        "  To capture a frame for calibration, press 'c',\n"
        "  If input comes from video, press any key for next frame\n"
        "  To finish capturing, press 'ESC' key and calibration starts.\n";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{@outfile |<none> | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       | false | Apply refind strategy }"
        "{zt       | false | Assume zero tangential distortion }"
        "{a        |       | Fix aspect ratio (fx/fy) to this value }"
        "{pc       | false | Fix the principal point at the center }"
        "{sc       | false | Show detected chessboard corners after calibration }";
}

/**
 */
static bool readDetectorParameters(std::string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    //fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}



/**
 */
static bool saveCameraParams(const std::string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    Mat rectificationMatrix, projectionMatrix;
    rectificationMatrix = Mat::eye(3, 3, CV_64F);
    projectionMatrix = Mat::zeros(3, 4, CV_64F);

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            projectionMatrix.at< double >(i, j) = cameraMatrix.at< double >(i, j);
        }
    }

    Mat croppedDistCoeffs = distCoeffs.colRange(0, 8);

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "distortion_model" << "plumb_bob";
    fs << "camera_name" << "raspicam";
    fs << "avg_reprojection_error" << totalAvgErr;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << croppedDistCoeffs;
    fs << "rectification_matrix" << rectificationMatrix;
    fs << "projection_matrix" << projectionMatrix;

    return true;
}

cv::Mat lastImage;
bool hasImage = false;

std::string directoryPath;

void imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img);
    lastImage = cv_img->image;
    hasImage = true;
}

Ptr<aruco::Board> board;
Ptr<aruco::CharucoBoard> charucoboard;
Ptr<aruco::Dictionary> dictionary;
Ptr<aruco::DetectorParameters> detectorParams;

struct arucoDetectResults_s
{
    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    Mat currentCharucoCorners, currentCharucoIds;
};

struct summaryArucoDetectResults_s
{
    vector< vector< vector< Point2f > > > corners;
    vector< vector< int > > ids;
    vector< Mat > imgs;
};


void detectMarkersProcessing(const Mat &_image, arucoDetectResults_s &detectedResults, const bool refindStrategy) {

    // detect markers
    aruco::detectMarkers(_image, dictionary, detectedResults.corners, detectedResults.ids, detectorParams, detectedResults.rejected);

    // refind strategy to detect more markers
    if(refindStrategy) aruco::refineDetectedMarkers(_image, board, detectedResults.corners, detectedResults.ids, detectedResults.rejected);

    // interpolate charuco corners
    if(detectedResults.ids.size() > 0)
        aruco::interpolateCornersCharuco(detectedResults.corners, detectedResults.ids, _image, charucoboard, detectedResults.currentCharucoCorners,
                                         detectedResults.currentCharucoIds);
}

void drawResults(Mat &image, arucoDetectResults_s detectResults, const vector< vector< vector< Point2f > > > allCorners) {
    if(detectResults.ids.size() > 0) aruco::drawDetectedMarkers(image, detectResults.corners);

    if(detectResults.currentCharucoCorners.total() > 0)
        aruco::drawDetectedCornersCharuco(image, detectResults.currentCharucoCorners, detectResults.currentCharucoIds);

    for(const auto& frameCorner : allCorners)
    {
        for(const auto& innerFrameCorner : frameCorner)
        {
            for(const auto& inFrameCorner : innerFrameCorner)
            {
                cv::circle(image, inFrameCorner, 1, cv::Scalar(255, 0, 0));
            }
        }
    }
}

void processArucoDetectResults(const Mat &image, const arucoDetectResults_s detectedResults, summaryArucoDetectResults_s &summaryDetectResults,
                               const int imgCounter) {
    bool allowCapture = (detectedResults.corners.size() > 0) && (detectedResults.ids.size() > 0);

    if (allowCapture) {
        cout << "Frame " << imgCounter << " captured" << endl;
        summaryDetectResults.corners.push_back(detectedResults.corners);
        summaryDetectResults.ids.push_back(detectedResults.ids);
        summaryDetectResults.imgs.push_back(image);
    }
    else {
        cout << "Frame rejected" << endl;
    }
}

void saveImage(const Mat &image, const bool saveCalibrationImages, const bool saveClean, int &imgCounter) {
        if (saveCalibrationImages) {
            cout << "Frame " << imgCounter << " saved" << endl;
            std::string imgPath = directoryPath + "/" + to_string(imgCounter) + ".png";
            if (saveClean) imwrite(imgPath.c_str(), image);
            else imwrite(imgPath.c_str(), image);
        }
        imgCounter++;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cv_calib");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    int squaresX = nhPriv.param("squares_x", 6);
    int squaresY = nhPriv.param("squares_y", 8);
    float squareLength = nhPriv.param("square_length", 0.021);
    float markerLength = nhPriv.param("marker_length", 0.013);
    int dictionaryId = nhPriv.param("dictionary_id", 4);

    bool saveCalibrationImages = nhPriv.param<bool>("save_images", true);
    bool saveClean = nhPriv.param<bool>("save_clean", false);
    bool refindStrategy = nhPriv.param<bool>("refind_markers", false);
    std::string calibrationSet = nhPriv.param<std::string>("calibration_set", "");
    std::string imagesFormat = nhPriv.param<std::string>("images_format", ".png");
    bool imageVerification = nhPriv.param<bool>("image_verification", false);
    int calibrationFlag = nhPriv.param("calibration_model", 0x04000);    
    std::string outputPath = nhPriv.param<std::string>("output_path", ".");

    // Get current datetime
    auto t = time(nullptr);
    auto tm = *localtime(&t);
    char time[50];
    std::strftime(time, 50, "%Y%m%d_%H%M%S", &tm);
    directoryPath = outputPath + "/calibration_" + time;

    // Make folder with timedate name
    std::string folderCreateCommand = "mkdir " + directoryPath;
    system(folderCreateCommand.c_str());
    
    // Get output filepath
    std::string outputFilePath = directoryPath + "/" + "calibration.yaml";
    cout << outputFilePath << endl;

    bool showChessboardCorners = true;

    float aspectRatio = 1;

    detectorParams = aruco::DetectorParameters::create();

    //image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_transport::ImageTransport it(nh);
    image_transport::ImageTransport itPriv(nhPriv);

    int waitTime = 10;
    ROS_INFO("Advertising charuco board image");
    auto pub = it.advertise("board", 1, true);
    ROS_INFO("Subscribing to image topic");
    auto sub = it.subscribe("image", 1, imageCallback /*, hints */);
    
    while(!hasImage)
    {
        ros::spinOnce();
    }
    
    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // create charuco board object
    charucoboard = aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    board = charucoboard.staticCast<aruco::Board>();

    cv::Mat boardImg(2048, 1536, CV_8UC3);
    charucoboard->draw(boardImg.size(), boardImg, 100);

    cv::imwrite("board.png", boardImg);
    cv_bridge::CvImage boardImgBridge;
    boardImgBridge.image = boardImg;
    pub.publish(boardImgBridge.toImageMsg());

    // collect data from each frame
    summaryArucoDetectResults_s summaryDetectResults;
    Size imgSize;

    int imgCounter = 1;

    if (calibrationSet == "") {
        while(hasImage) {
            Mat image, imageCopy, imageToSave;
            image = lastImage.clone();

            arucoDetectResults_s detectedResults;
            detectMarkersProcessing(image, detectedResults, refindStrategy);

            // draw results
            image.copyTo(imageCopy);
            drawResults(imageCopy, detectedResults, summaryDetectResults.corners);

            putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                    Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

            imshow("Out", imageCopy);

            char key = (char)waitKey(waitTime);
            if (key == 27) break;
            if (key == 'r') {
                if (refindStrategy) refindStrategy = false;
                else refindStrategy = true;
            }
            if(key == 'c' && (detectedResults.ids.size() > 0)) {
                if (imgSize.height == 0) imgSize = image.size();
                processArucoDetectResults(image, detectedResults, summaryDetectResults, imgCounter);
                saveImage(image, saveCalibrationImages, saveClean, imgCounter);
            }
            ros::spinOnce();
            if (ros::isShuttingDown())
            {
                return 0;
            }
        }

        if(summaryDetectResults.ids.size() < 4) {
            cerr << "Not enough captures for calibration" << endl;
            return 0;
        }

        cvDestroyWindow("Out");
    } else {
        cout << "Calibrate from directory" << endl;
        
        Mat image, imageCopy;
        DIR *dir;
        struct dirent *ent;
        
        if ((dir = opendir (calibrationSet.c_str())) != NULL) {
            while ((ent = readdir (dir)) != NULL) {
                std::string file_name = ent->d_name;

                /*Check the file is an image*/
                if (std::search(file_name.cbegin(), file_name.cend(), imagesFormat.cbegin(), imagesFormat.cend()) != file_name.cend()) {
                    arucoDetectResults_s detectedResults;
                    if (imgSize.height == 0) imgSize = image.size();

                    file_name = calibrationSet + file_name;
                    image = cv::imread(file_name);
                    detectMarkersProcessing(image, detectedResults, refindStrategy);

                    if (imageVerification) {
                        while (true) {
                            image.copyTo(imageCopy);
                            putText(imageCopy, "Press 'n' to use this image for calibration, 'n' to skip. 'ESC' to exit",
                                    Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
                            drawResults(imageCopy, detectedResults, summaryDetectResults.corners);

                            imshow("Checked image", imageCopy);

                            char key = (char)waitKey();
                            if (key == 'y') {
                                processArucoDetectResults(image, detectedResults, summaryDetectResults, imgCounter);
                                saveImage(image, saveCalibrationImages, saveClean, imgCounter);
                                break;
                            } else if (key == 'n') {
                                break;
                            } else if (key == 27) {
                                return 0;
                            } else { 
                                continue;
                            }
                        }
                    } else {
                        processArucoDetectResults(image, detectedResults, summaryDetectResults, imgCounter);
                        saveImage(image, saveCalibrationImages, saveClean, imgCounter);
                    }
                }
            }
            closedir (dir);
        } else {
            /* could not open directory */
            cout << "Error while opening calibration image directory" << endl;
            return EXIT_FAILURE;
        }
        cvDestroyWindow("Checked image");
    }

    cout << "Calibrating..." << endl;

    Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    if(calibrationFlag & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(summaryDetectResults.corners.size());
    for(unsigned int i = 0; i < summaryDetectResults.corners.size(); i++) {
        markerCounterPerFrame.push_back((int)summaryDetectResults.corners[i].size());
        for(unsigned int j = 0; j < summaryDetectResults.corners[i].size(); j++) {
            allCornersConcatenated.push_back(summaryDetectResults.corners[i][j]);
            allIdsConcatenated.push_back(summaryDetectResults.ids[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
                                              distCoeffs, noArray(), noArray(), calibrationFlag);

    // prepare data for charuco calibration
    int nFrames = (int)summaryDetectResults.corners.size();
    vector< Mat > allCharucoCorners;
    vector< Mat > allCharucoIds;
    vector< Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        Mat currentCharucoCorners, currentCharucoIds;
        aruco::interpolateCornersCharuco(summaryDetectResults.corners[i], summaryDetectResults.ids[i], summaryDetectResults.imgs[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(summaryDetectResults.imgs[i]);
    }

    if(allCharucoCorners.size() < 4) {
        cerr << "Not enough corners for calibration" << endl;
        return 0;
    }

    // calibrate camera using charuco
    repError =
        aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlag);

    bool saveOk =  saveCameraParams(outputFilePath, imgSize, aspectRatio, calibrationFlag,
                                    cameraMatrix, distCoeffs, repError);
    if(!saveOk) {
        cerr << "Cannot save output file" << endl;
        return 0;
    }

    cout << "Reprojection error: " << repError << endl;
    cout << "Reprojection error for aruco: " << arucoRepErr << endl;
    cout << "Calibration saved to " << outputFilePath << endl;
    cout << "Check undistorted images from camera. Press esc to exit." << endl;

    while(hasImage) {
        Mat image, imageUndistorted;
        image = lastImage.clone();
        imageUndistorted = image.clone();

        undistort(image, imageUndistorted, cameraMatrix, distCoeffs);

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
