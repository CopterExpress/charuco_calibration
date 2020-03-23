#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <functional>

namespace charuco_calibration
{

/** Camera calibrator parameter list */
struct CalibratorParams
{
    /** Number of chessboard squares along the X axis */
    int squaresX;
    /** Number of chessboard squares along the Y axis*/
    int squaresY;
    /** Size of a chessboard square in meters */
    float squareLength;
    /** Size of an ArUco marker in meters */
    float markerLength;
    /** Dictionary ID to use for markers (see https://docs.opencv.org/3.2.0/d9/d6a/group__aruco.html#gac84398a9ed9dd01306592dd616c2c975 for details)*/
    int dictionaryId;
    /** Calibration flags (see https://docs.opencv.org/3.2.0/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d for details)*/
    int calibrationFlags;
    /** Aspect ratio (if CALIB_FIX_ASPECT_RATIO is set) */
    double aspectRatio;
    /** Should we perform ArUco detection refinement? */
    bool performRefinement;
    /** Should we draw markers that we have already seen? */
    bool drawHistoricalMarkers;

    CalibratorParams() : squaresX(6), squaresY(8),
        squareLength(0.021), markerLength(0.013),
        dictionaryId(4), calibrationFlags(cv::CALIB_RATIONAL_MODEL),
        aspectRatio(1.0), performRefinement(false),
        drawHistoricalMarkers(true) {}
};

/** Detection result for a single image */
struct CalibratorDetectionResult
{
    /** Source image */
    const cv::Mat sourceImage;
    /** IDs of detected markers */
    std::vector<int> ids;
    /** Detected marker corners */
    std::vector<std::vector<cv::Point2f>> corners;
    /** Detected ChAruco corners */
    cv::Mat charucoCorners;
    /** Detected ChAruco IDs */
    cv::Mat charucoIds;
    /** Check whether this is a valid result for calibration */
    bool isValid() const { return ids.size() > 0 && corners.size() > 0; }
    CalibratorDetectionResult(const cv::Mat& sourceImage) : sourceImage(sourceImage), ids(), corners() {}
};

/** Camera calibration result */
struct CalibrationResult
{
    /* Camera resolution */
    cv::Size imgSize;
    /* Intrinsic camera matrix */
    cv::Mat cameraMatrix;
    /* Lens distorion coefficients */
    cv::Mat distCoeffs;
    /* Chessboard reprojection error */
    double reprojectionError;
    /* ArUco reprojection error */
    double arucoReprojectionError;
    /* Rotation vectors */
    std::vector<cv::Mat> rvecs;
    /* Translation vectors */
    std::vector<cv::Mat> tvecs;

    /* Is this calibration successful? */
    bool isValid;
};

enum LogLevel
{
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
};

using CalibratorLogFunction = std::function<void(LogLevel, const std::string&)>;

class Calibrator
{
private:
    CalibratorLogFunction calibLogger;

public:
    /** Console logging function (the default one) */
    static void consoleLogFunction(LogLevel logLevel, const std::string& message);
    /** Null logging function (no actual logging will be performed) */
    static void nullLogFunction(LogLevel ll, const std::string& msg)
    {
        std::ignore = ll;
        std::ignore = msg;
    };
    /** Set logging function */
    void setLogger(const CalibratorLogFunction& logFcn)
    {
        calibLogger = logFcn;
    }

private:

    cv::Ptr<cv::aruco::Dictionary> arucoDictionary;
    cv::Ptr<cv::aruco::CharucoBoard> charucoBoard;

    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
    std::vector<std::vector<int>> allIds;
    std::vector<cv::Mat> allImgs;

    CalibratorLogFunction logFunction;

public:
    /** Calibrator parameters: number of chessboard squares, ArUco sizes, etc */
    CalibratorParams params;
    /** ArUco detector parameters */
    cv::Ptr<cv::aruco::DetectorParameters> arucoDetectorParams;
    /** Update internal state based on current params value; call this after changing params */
    void applyParams();

    Calibrator();
    ~Calibrator() {};

    /** Get cv::Mat with the board image */
    cv::Mat getBoardImage(int width, int height, int margin=0)
    {
        cv::Mat boardImg(height, width, CV_8UC1);
        charucoBoard->draw(boardImg.size(), boardImg, margin);
        return boardImg;
    }

    /** Find board on the image */
    CalibratorDetectionResult processImage(const cv::Mat& image);

    /** Show image with detected markers (if applicable) */
    cv::Mat drawDetectionResults(const CalibratorDetectionResult& detectionResult);

    /** Add image to the calibration list */
    bool addToCalibrationList(const CalibratorDetectionResult& detectionResult)
    {
        if (detectionResult.isValid())
        {
            allCorners.push_back(detectionResult.corners);
            allIds.push_back(detectionResult.ids);
            allImgs.push_back(detectionResult.sourceImage);
            return true;
        }
        return false;
    }

    std::vector<cv::Mat>& getStoredCalibrationImages()
    {
        return allImgs;
    }

    CalibrationResult performCalibration();
};

}
