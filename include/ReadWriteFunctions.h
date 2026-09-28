#ifndef _READWRITE_FUNCTIONS_
#define _READWRITE_FUNCTIONS_


#include <opencv2/opencv.hpp>

bool readMonoCameraCalibParameters(std::string filename, cv::Mat &camMatrix, cv::Mat & distCoeffs);

bool readStereoCameraParameters(std::string filename, cv::Mat &camLMatrix, cv::Mat & distLCoeffs, cv::Mat &camRMatrix, cv::Mat & distRCoeffs, cv::Mat &stereoMapL_x, cv::Mat &stereoMapL_y, cv::Mat &stereoMapR_x, cv::Mat &stereoMapR_y, cv::Mat &Q);

bool readDisparityParameters(std::string filename, int &numDisparities, int &blockSize , int &preFilterType, int &preFilterSize, int &preFilterCap, int &minDisparity , int &textureThreshold, int &uniquenessRatio, int &speckleRange, int &speckleWindowSize, int &disp12MaxDiff);

bool readColorParameters(std::string filename, int& iLowH, int& iHighH, int& iLowS, int& iHighS, int& iLowV, int& iHighV);

bool readCameraChessboardCalibrationParameters(std::string filename, std::vector<cv::Point3f>& objPoints, std::vector<cv::Point2f>& imagePoints, cv::Mat& cameraChessboardTransform);

bool writeMonoCameraParameters(std::string filename, cv::Mat camMatrix, cv::Mat distCoeffs);

bool writeColorParameters(std::string filename, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV);

bool writeStereoCameraParameters(std::string filename, cv::Mat camLMatrix, cv::Mat distLCoeffs, cv::Mat camRMatrix, cv::Mat distRCoeffs, cv::Mat stereoMapL_x, cv::Mat stereoMapL_y, cv::Mat stereoMapR_x, cv::Mat stereoMapR_y, cv::Mat Q);

bool writeDisparityParameters(std::string filename, int numDisparities, int blockSize , int preFilterType, int preFilterSize, int preFilterCap, int minDisparity , int textureThreshold, int uniquenessRatio, int speckleRange, int speckleWindowSize, int disp12MaxDiff);

bool writeCamRobotCalibrationParameters(std::string filename, cv::Mat robotCamTransform);

bool writeCameraChessboardCalibrationParameters(std::string filename, std::vector<cv::Point3f> objPoints, std::vector<cv::Point2f> imagePoints, cv::Mat cameraChessboardTransform);
#endif