#include <chrono>
#include <thread>
#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>

#include "Kinematics.h"
#include "DynamixelHandler.h"

#define CAM_PARAMS_FILENAME "./data/microsoft_livecam_hd3000.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params_data.xml"
#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5
#define AREA_THRESOLD 1000
#define ROBOT_L1 5
#define ROBOT_L2 6
#define HEIGHT2CM 24
#define WIDTH2CM 33

using namespace cv;
using namespace std;

DynamixelHandler _oDxlHandler;
std::string _robotDxlPortName = "/dev/ttyUSB0";
float _robotDxlProtocol = 2.0;
int _robotDxlBaudRate = 1000000;


void initRobot(DynamixelHandler& dxlHandler, std::string portName, float protocol, int baudRate)
{
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	dxlHandler.setDeviceName(portName);
	dxlHandler.setProtocolVersion(protocol);
	dxlHandler.openPort();
	dxlHandler.setBaudRate(baudRate);
	dxlHandler.enableTorque(true);
	std::cout << std::endl;
}

void closeRobot(DynamixelHandler& dxlHandler)
{
	dxlHandler.enableTorque(false);
	dxlHandler.closePort();
}

bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat & distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open the camera parameter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	
	return true;
}

bool readColorParameters(std::string filename, int& iLowH, int& iHighH, int& iLowS, int& iHighS, int& iLowV, int& iHighV)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open the color paramter file storage: " <<  filename << " !"<< std::endl;
		return false;
	}
	
	fs["lowH"] >> iLowH;
	fs["highH"] >> iHighH;
	
	fs["lowS"] >> iLowS;
	fs["highS"] >> iHighS;
	
	fs["lowV"] >> iLowV;
	fs["highV"] >> iHighV;
	
	return true;
}


int main(int argc, char** argv)
{
	// initializes main parameters
	float L1 = ROBOT_L1;
	float L2 = ROBOT_L2;
	float qpen = deg2rad(-90); // in rad
	std::string sCameraParamFilename = CAM_PARAMS_FILENAME;
	std::string sColorParamFilename = COLOR_PARAMS_FILENAME;
	float fFPS = FPS;
	int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
	int iAreaThresold = AREA_THRESOLD;
	
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":c:f:s:a:i:p:l:m:")) != -1)
	{
		switch (opt)
		{
			case 'c':
				sColorParamFilename = optarg;
				break;
			case 'f':
				fFPS = atof(optarg); 
				break;
			case 'p':
				qpen = atof(optarg); 
				break;
			case 'l':
				L1 = atof(optarg); 
				break;
			case 'm':
				L2 = atof(optarg); 
				break;
			case 's':
				iStructuralElementSize = atoi(optarg);
				break;
			case 'a':
				iAreaThresold = atoi(optarg);
				break;
			case 'i':
				sCameraParamFilename = optarg;
				break;
			case '?':
				if (optopt == 'c' || optopt == 'f' || optopt == 's' || optopt == 'a' || optopt == 'p' || optopt == 'l' || optopt == 'm' || optopt == 'i')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
				return 1;
			default:
				abort ();
		}
	}
	
	// Initializes the robot
	initRobot(_oDxlHandler, _robotDxlPortName, _robotDxlProtocol, _robotDxlBaudRate);
	
	
	// reads color parameters from the file storage
	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
	bool isColorParamsSet = readColorParameters(sColorParamFilename, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
	
	// checks if the color parameters were successfully read
	if (!isColorParamsSet)
	{
		std::cout << "[ERROR] Color parameters could not be loaded!" << std::endl;
		return -1;
	}
	
	// distorted/undistorted image
	bool bIsImageUndistorted = true;
	
	// reads camera intrinsic parameters
	cv::Mat cameraMatrix, distCoeffs;
	bool isCamParamsSet = readCameraParameters(sCameraParamFilename, cameraMatrix, distCoeffs);
	
	// checks if the camera parameters were successfully read
	if (!isCamParamsSet)
	{
		std::cout << "[WARNING] Camera intrinsic parameters could not be loaded!" << std::endl;
	}
	 
	// creates a camera grabber
	VideoCapture cap(0, cv::CAP_V4L2); //capture the video from webcam
	
	// checks if the camera was successfully opened
	if ( !cap.isOpened() )  // if not success, exit program
	{
		cout << "[ERROR] Could not open the camera!" << endl;
		return -1;
	}

	// inits previous x,y location of the ball
	int iLastX = -1; 
	int iLastY = -1;

	// captures a temporary image from the camera
	Mat imgTmp;
	cap.read(imgTmp); 

	// main loop launched every FPS
	while (true)
	{
		// creates a black image with the size as the camera output
		Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );
		
		// reads a new frame from video
		cv::Mat imgOriginal;
		bool bSuccess = cap.read(imgOriginal);

		// checks if a new frame was grabbed
		if (!bSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a new frame from video stream" << std::endl;
			break;
		}
		
		if (bIsImageUndistorted && isCamParamsSet)
		{
			cv::Mat temp = imgOriginal.clone();
			cv::undistort(temp, imgOriginal, cameraMatrix, distCoeffs);
		}

		// converts the captured frame from BGR to HSV
		cv::Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); 

		// thresholds the image based on the trackbar values
		cv::Mat imgThresholded;
		inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); 

		// applies morphological opening (removes small objects from the foreground)
		cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iStructuralElementSize, iStructuralElementSize)) );
		cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iStructuralElementSize, iStructuralElementSize)) ); 

		// applies morphological closing (removes small holes from the foreground)
		cv::dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iStructuralElementSize, iStructuralElementSize)) ); 
		cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iStructuralElementSize, iStructuralElementSize)) );

		// calculates the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= iAreaThresold, considers that the there are no object in the image and it's because of the noise, the area is not zero 
		int posX, posY;
		
		if (dArea > iAreaThresold)
		{
			// calculates the position of the ball
			posX = dM10 / dArea;
			posY = dM01 / dArea;        

			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
			{
				// draww a red line from the previous point to the current point
				line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
			}

			// stores the current position for enxt frame
			iLastX = posX;
			iLastY = posY;
		}

		// displays the thresholded image
		imshow("Thresholded Image", imgThresholded); 

		// adds a cross at the centre of the image
		cv::drawMarker(imgOriginal, cv::Point(imgTmp.size().width/2, imgTmp.size().height/2), 10, cv::MARKER_CROSS, cv::LINE_8);
		
		// shows the original image with the tracking (red) lines
		imgOriginal = imgOriginal + imgLines;
		imshow("Original", imgOriginal); 
		
		// converts posX, posY in mm in the world reference frame
		float img_width = imgTmp.size().width;
		float img_height = imgTmp.size().height;
		float x = (posY-img_height/2)/img_height*HEIGHT2CM; 
		float y = (posX-img_width/2)/img_width*WIDTH2CM; 
		
		std::cout << "(pixel -> cm) = (" << posX << ", " << posY << ") - > (" << x << ", " << y << ")" << std::endl;
		
		// Computes IK
		std::vector<float> qi = computeInverseKinematics(x, y, L1, L2);
		
		// Computes FK
		//computeForwardKinematics(qi[1], qi[2], L1, L2);
	
		// Sends the target joint values received only if there is at least a solution
		if (qi.size() >= 3)
		{
			std::vector<float> vTargetJointPosition;
				vTargetJointPosition.push_back(qi[1]); 
				vTargetJointPosition.push_back(qpen); 
				vTargetJointPosition.push_back(qi[2]); 
			_oDxlHandler.sendTargetJointPosition(vTargetJointPosition);
		}

		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] esc key is pressed by user -> Shutting down!" << std::endl;
			break; 
		}
		if (key == 'u')
		{
			bIsImageUndistorted = !bIsImageUndistorted;
			std::cout << "[INFO] Image undistorted: " <<  bIsImageUndistorted<< std::endl;
		}
	}
	
	
	
	
		
	
	// Closes robot connection
	_oDxlHandler.closePort();
		
	return 0;
}