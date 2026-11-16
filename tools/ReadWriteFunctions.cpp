#include "ReadWriteFunctions.h"



bool readMonoCameraCalibParameters(std::string filename, cv::Mat &camMatrix, cv::Mat & distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (READ) the mono camera calibration parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	
	// releases the reader
	fs.release();
	
	return true;
}

bool readStereoCameraParameters(std::string filename, cv::Mat &camLMatrix, cv::Mat & distLCoeffs, cv::Mat &camRMatrix, cv::Mat & distRCoeffs, cv::Mat &stereoMapL_x, cv::Mat &stereoMapL_y, cv::Mat &stereoMapR_x, cv::Mat &stereoMapR_y, cv::Mat &Q)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (READ) the stereo camera parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["Left_Camera_Matrix"] >> camLMatrix;
	fs["Left_Camera_Distortion"] >> distLCoeffs;
	fs["Right_Camera_Matrix"] >> camRMatrix;
	fs["Right_Camera_Distortion"] >> distRCoeffs;
	fs["Left_Stereo_Map_x"]  >> stereoMapL_x;
	fs["Left_Stereo_Map_y"]  >> stereoMapL_y;
	fs["Right_Stereo_Map_x"]  >> stereoMapR_x;
	fs["Right_Stereo_Map_y"]  >> stereoMapR_y;
	fs["Q"]  >> Q;
	
	// releases the reader
	fs.release();
	
	return true;
}

bool readDisparityParameters(std::string filename, int &numDisparities, int &blockSize , int &preFilterType, int &preFilterSize, int &preFilterCap, int &minDisparity , int &textureThreshold, int &uniquenessRatio, int &speckleRange, int &speckleWindowSize, int &disp12MaxDiff)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (READ) the disparity parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["numDisparities"] >> numDisparities;
	fs["blockSize"] >> blockSize;
	fs["preFilterType"] >> preFilterType;
	fs["preFilterSize"] >> preFilterSize;
	fs["preFilterCap"] >> preFilterCap;
	fs["minDisparity"] >> minDisparity;
	fs["textureThreshold"] >> textureThreshold;
	fs["uniquenessRatio"] >> uniquenessRatio;
	fs["speckleRange"] >> speckleRange;
	fs["speckleWindowSize"] >> speckleWindowSize;
	fs["disp12MaxDiff"] >> disp12MaxDiff;
	
	// releases the reader
	fs.release();
	
	return true;
}

bool readColorParameters(std::string filename, int& iLowH, int& iHighH, int& iLowS, int& iHighS, int& iLowV, int& iHighV)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (READ) the color parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["lowH"] >> iLowH;
	fs["highH"] >> iHighH;
	fs["lowS"] >> iLowS;
	fs["highS"] >> iHighS;
	fs["lowV"] >> iLowV;
	fs["highV"] >> iHighV;
	
	// releases the reader
	fs.release();
	
	return true;
}

bool readCameraChessboardCalibrationParameters(std::string filename, std::vector<cv::Point3f>& objPoints, std::vector<cv::Point2f>& imagePoints, cv::Mat& cameraChessboardTransform)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (READ) the camera/chessboard calibration parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["objPoints"] >> objPoints;
	fs["imagePoints"] >> imagePoints;
	fs["cameraChessboardTransform"] >> cameraChessboardTransform;
	
	// releases the reader
	fs.release();
	
	return true;
}

bool writeMonoCameraParameters(std::string filename, cv::Mat camMatrix, cv::Mat distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (WRITE) the mono camera parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs << "camera_matrix" << camMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	
	// releases the writer
	fs.release();
	
	return true;
}

bool writeColorParameters(std::string filename, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (WRITE) the color parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs << "lowH" << iLowH;
	fs << "highH" << iHighH;
	fs << "lowS" << iLowS;
	fs << "highS" << iHighS;
	fs << "lowV" << iLowV;
	fs << "highV" << iHighV;
	
	// releases the writer
        fs.release();

	return true;
}

bool writeStereoCameraParameters(std::string filename, cv::Mat camLMatrix, cv::Mat distLCoeffs, cv::Mat camRMatrix, cv::Mat distRCoeffs, cv::Mat stereoMapL_x, cv::Mat stereoMapL_y, cv::Mat stereoMapR_x, cv::Mat stereoMapR_y, cv::Mat Q)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (WRITE) the stereo camera parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs << "Left_Camera_Matrix" << camLMatrix;
	fs << "Left_Camera_Distortion" << distLCoeffs;
	fs << "Right_Camera_Matrix" << camRMatrix;
	fs << "Right_Camera_Distortion" << distRCoeffs;
	fs << "Left_Stereo_Map_x" << stereoMapL_x;
	fs << "Left_Stereo_Map_y" << stereoMapL_y;
	fs << "Right_Stereo_Map_x" << stereoMapR_x;
	fs << "Right_Stereo_Map_y" << stereoMapR_y;
	fs << "Q" << Q;
	
	// releases the writer
	fs.release();
	
	return true;
}

bool writeDisparityParameters(std::string filename, int numDisparities, int blockSize , int preFilterType, int preFilterSize, int preFilterCap, int textureThreshold, int uniquenessRatio, int speckleRange, int speckleWindowSize, int disp12MaxDiff, int minDisparity )
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (WRITE) the disparity parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs << "numDisparities" << numDisparities;
	fs << "blockSize" << blockSize;
	fs << "preFilterType" << preFilterType;
	fs << "preFilterSize" << preFilterSize;
	fs << "preFilterCap" << preFilterCap;
	fs << "minDisparity" << minDisparity;
	fs << "textureThreshold" << textureThreshold;
	fs << "uniquenessRatio" << uniquenessRatio;
	fs << "speckleRange" << speckleRange;
	fs << "speckleWindowSize" << speckleWindowSize;
	fs << "disp12MaxDiff" << disp12MaxDiff;
	
	// releases the writer
	fs.release();
	
	return true;
}


bool writeCamRobotCalibrationParameters(std::string filename, cv::Mat robotCamTransform)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (WRITE) the camera/robot calibration parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs << "robotCamTransform" << robotCamTransform;
	
	// releases the writer
	fs.release();
	
	return true;
}


bool writeCameraChessboardCalibrationParameters(std::string filename, std::vector<cv::Point3f> objPoints, std::vector<cv::Point2f> imagePoints, cv::Mat cameraChessboardTransform)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open (WRITE) the camera/chessboard calibration parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs << "objPoints" << objPoints;
	fs << "imagePoints" << imagePoints;
	fs << "cameraChessboardTransform" << cameraChessboardTransform;
	
	// releases the writer
	fs.release();
	
	return true;
}