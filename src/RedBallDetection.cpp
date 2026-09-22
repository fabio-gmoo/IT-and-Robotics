#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#define CAM_PARAMS_FILENAME "./data/microsoft_livecam_hd3000.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params_data.xml"
#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5

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

bool writeColorParameters(std::string filename, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		std::cout << "[ERROR] Could not open the file storage: " <<  filename << " !"<< std::endl;
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

int main(int argc, char** argv)
{	
	// initializes main parameters
	std::string sCameraParamFilename = CAM_PARAMS_FILENAME;
	std::string sColorParamFilename = COLOR_PARAMS_FILENAME;
	int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
	float fFPS = FPS;
	
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":i:f:o:s:")) != -1)
	{
		switch (opt)
		{
			case 'o':
				sColorParamFilename = optarg;
				break;
			case 'f':
				fFPS = atof(optarg); 
				break;
			case 'i':
				sCameraParamFilename = optarg;
				break;
			case 's':
				iStructuralElementSize = atoi(optarg);
				break;
			case '?':
				if (optopt == 'o' || optopt == 'f' || optopt == 'i' || optopt == 's')
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
	cv::VideoCapture cap(0, cv::CAP_V4L2);
	
	// checks if the camera was successfully opened
	if (!cap.isOpened())
	{
		std::cout << "[ERROR] Cannot open the webcam" << std::endl;
		return 1;
	}
	
	cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); //create a window called "Control"

	// sets min/max value for HSV color representation
	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0; 
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	// creates trackbars in "Control" window
	cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control", &iHighH, 179);

	cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control", &iHighS, 255);

	cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cv::createTrackbar("HighV", "Control", &iHighV, 255);

	while (true)
	{
		cv::Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Cannot read a frame from video stream" << std::endl;
			break;
		}
		
		if (bIsImageUndistorted && isCamParamsSet)
		{
			cv::Mat temp = imgOriginal.clone();
			cv::undistort(temp, imgOriginal, cameraMatrix, distCoeffs);
		}
		
		//Convert the captured frame from BGR to HSV
		cv::Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); 

		//Threshold the image based on the trackbar values
		cv::Mat imgThresholded;
		inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); 

		//morphological opening (remove small objects from the foreground)
		cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
		cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 

		//morphological closing (fill small holes in the foreground)
		cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
		cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );

		cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
		cv::imshow("Original", imgOriginal); //show the original image

		// waits for awhile depending on the FPS value
		// checks if ESC was pressed to exit
		char key = (char)cv::waitKey(1000.0/fFPS);
		//wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		if (key == 27)
		{
			std::cout << "[INFO] esc key is pressed by user -> Shuting down!" << std::endl;
			break; 
		}
		if (key == 'u')
		{
			bIsImageUndistorted = !bIsImageUndistorted;
			std::cout << "[INFO] Image undistorted: " <<  bIsImageUndistorted<< std::endl;
		}
		if (key == 's')
		{
			writeColorParameters(sColorParamFilename, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
			std::cout << "[INFO] Color parameters saved to file: " <<  sColorParamFilename << std::endl;
		}
		
		
	}

	return 0;
}
