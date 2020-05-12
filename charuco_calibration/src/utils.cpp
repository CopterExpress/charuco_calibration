#include "utils.h"

#include <sensor_msgs/distortion_models.h>

namespace
{

std::string loggerName("utils");

}

namespace charuco_calibration
{

bool readDetectorParameters(ros::NodeHandle& nh, cv::Ptr<cv::aruco::DetectorParameters> &params) {
#define GET_PARAM(paramName) {params->paramName = nh.param(#paramName, params->paramName); \
                              ROS_INFO_STREAM_NAMED(loggerName.c_str(), #paramName " set to " << params->paramName);}
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
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 3)) || (CV_VERSION_MAJOR > 3)
    GET_PARAM(cornerRefinementMethod);
#else
    // Older OpenCV versions only have doCornerRefinement
    int cornerRefinementMethod = nh.param("cornerRefinementMethod", 0);
    if (cornerRefinementMethod > 1)
    {
        ROS_WARN_STREAM_NAMED(loggerName.c_str(), "cornerRefinementMethod set to " << cornerRefinementMethod << ", but current OpenCV version only supports subpixel refinement");
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

bool readCalibrationFlags(ros::NodeHandle& nh, int& calibrationFlags)
{
    if (!nh.hasParam("calibration_flags"))
    {
        return false;
    }
    calibrationFlags = 0;
#define GET_FLAG(flag_name) { bool flagSet = nh.param("calibration_flags/" #flag_name, false); \
                              calibrationFlags |= flagSet ? cv::flag_name : 0; \
                              ROS_INFO_STREAM_NAMED(loggerName.c_str(), "Calibration flag " #flag_name " set to " << flagSet);}
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
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 3)) || (CV_VERSION_MAJOR > 3)
    GET_FLAG(CALIB_FIX_TANGENT_DIST);
#endif
#if ((CV_VERSION_MAJOR == 3) && ((CV_VERSION_MINOR == 4) && (CV_VERSION_REVISION >= 1) || (CV_VERSION_MINOR > 4))) || (CV_VERSION_MAJOR > 3)
    GET_FLAG(CALIB_USE_EXTRINSIC_GUESS);
#endif
#undef GET_FLAG
    return true;
}

void readCalibratorParams(ros::NodeHandle& nh, charuco_calibration::Calibrator& calibrator)
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
        ROS_WARN_NAMED(loggerName.c_str(), "Could not retrieve calibration_flags from parameter server");
        calibrator.params.calibrationFlags = nh.param<int>("calibration_flags_mask", cv::CALIB_RATIONAL_MODEL);
    }
    else
    {
        ROS_INFO_STREAM_NAMED(loggerName.c_str(), "Calibration flags set to " << calibrator.params.calibrationFlags);
    }
    if (std::abs(calibrator.params.aspectRatio - 1.0) > 0.01)
    {
        if (!(calibrator.params.calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO))
        {
            ROS_WARN_STREAM_NAMED(loggerName.c_str(), "Aspect ratio is " << calibrator.params.aspectRatio << ", but CALIB_FIX_ASPECT_RATIO is not set");
        }
    }
    calibrator.applyParams();
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
std::ostream& saveCameraInfo(std::ostream& output, charuco_calibration::CalibrationResult& result)
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

void setLoggerName(const std::string& loggerName)
{
    ::loggerName = loggerName;
}

}
