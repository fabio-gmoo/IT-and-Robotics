#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>

#define CAM_PARAMS_FILENAME "./data/microsoft_livecam_hd3000.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params_data.xml"
#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5
#define AREA_THRESOLD 1000

using namespace cv;
using namespace std;

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

 int main( int argc, char** argv )
 {
	// initializes main parameters
	std::string sCameraParamFilename = CAM_PARAMS_FILENAME;
	std::string sColorParamFilename = COLOR_PARAMS_FILENAME;
	float fFPS = FPS;
	int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
	int iAreaThresold = AREA_THRESOLD;
	 
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":c:f:s:a:i:")) != -1)
	{
		switch (opt)
		{
			case 'c':
				sColorParamFilename = optarg;
				break;
			case 'f':
				fFPS = atof(optarg); 
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
				if (optopt == 'c' || optopt == 'f' || optopt == 's' || optopt == 'a')
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

	// creates a black image with the size as the camera output
	Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );

	// main loop launched every FPS
	while (true)
	{
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
		if (dArea > iAreaThresold)
		{
			// calculates the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;        

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

		// shows the original image with the tracking (red) lines
		imgOriginal = imgOriginal + imgLines;
		imshow("Original", imgOriginal); 

		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] esc key is pressed by user -> Shuting down!" << std::endl;
			break; 
		}
		if (key == 'u')
		{
			bIsImageUndistorted = !bIsImageUndistorted;
			std::cout << "[INFO] Image undistorted: " <<  bIsImageUndistorted<< std::endl;
		}
	}

	return 0;
}