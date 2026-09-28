#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include "ReadWriteFunctions.h"

#define MONO_CALIB_PARAMS_FILENAME "./data/camera_calibration_params.xml"
#define FPS 20
#define CAM_INDEX 0
#define NB_FRAMES 25
#define SQUARE_SIZE 23.4
#define CHECKERBOARD_WIDTH 6
#define CHECKERBOARD_HEIGHT 9
#define RESOLUTION_MAX 800
 

int main( int argc, char** argv )
{
	// initializes main parameters 
	float fFPS = FPS;
	int iCamIndex = CAM_INDEX;
	int iNbFrames = NB_FRAMES;
	float fSquareSize = SQUARE_SIZE; // in mm
	int iCheckerBoardWidth = CHECKERBOARD_WIDTH;
	int iCheckerBoardHeight = CHECKERBOARD_HEIGHT;
	std::string sMonoCalibParamFilename = MONO_CALIB_PARAMS_FILENAME;
	int iMaxVideoResolution = RESOLUTION_MAX;
	
	
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":o:f:c:n:s:w:h:i:j:r:")) != -1)
	{
		switch (opt)
		{
			case 'o':
				sMonoCalibParamFilename = optarg;
				break;
			case 'f':
				fFPS = atof(optarg); 
				break;
			case 'c':
				iCamIndex = atoi(optarg);
				break;
			case 'n':
				iNbFrames = atoi(optarg);
				break;
			case 's':
				fSquareSize = atof(optarg);
				break;
			case 'w':
				iCheckerBoardWidth = atoi(optarg);
				break;
			case 'h':
				iCheckerBoardHeight = atoi(optarg);
				break;
			case 'r':
				iMaxVideoResolution = atoi(optarg);
				break;
			case '?':
				if (optopt == 'o' || optopt == 'f' || optopt == 'c' || optopt == 'n' || optopt == 's' || optopt == 'w' || optopt == 'h'|| optopt == 'i'|| optopt == 'j'|| optopt == 'r')
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

	// creates camera grabbers for the left and right cameras
	std::cout << "[INFO] Opening camera videostreams...";
	cv::VideoCapture cam(iCamIndex, cv::CAP_V4L2);
	
	// changes image resolution to maximum (e.g. 1920x1080 if possible)
	cam.set(cv::CAP_PROP_FRAME_HEIGHT, iMaxVideoResolution); cam.set(cv::CAP_PROP_FRAME_WIDTH, iMaxVideoResolution);
		
	// checks if the camera was successfully opened
	if (!cam.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "[ERROR] Could not open the camera!" << std::endl;
		return 1;
	}
	std::cout << "OK!" << std::endl;
	
	// gets image resolution for info
	std::cout << "[INFO] Left camera resolution: " << cam.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cam.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
		
	// inits display
	cv::namedWindow("Frames", cv::WINDOW_NORMAL);
	cv::resizeWindow("Frames", 640, 480);
	
	// defines the world coordinates for 3D points (in mm)
	std::vector<cv::Point3f> objp;
	for(int i{0}; i<iCheckerBoardHeight; i++)
	{
		for(int j{0}; j<iCheckerBoardWidth; j++)
			objp.push_back(cv::Point3f(j*fSquareSize, i*fSquareSize, 0));
	}
	
	// main loop launched every FPS
	int iCount = 0;
	bool bIsCalibrated = false;
	bool bIsUndistort = false;
	cv::Mat camMatrix, distCoeffs;
	cv::Mat frame, gray, view;
	std::vector<std::vector<cv::Point3f> > objpoints; // vector to store vectors of 3D points for each checkerboard image
	std::vector<std::vector<cv::Point2f> > imgpoints; // vector to store vectors of 2D points for each checkerboard image
	
	while (true)
	{
		// reads a new frame from left & right cameras
		bool bCamSuccess = cam.read(frame);
			
		// checks if a new frame was grabbed 
		if (!bCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a frame from video stream" << std::endl;
			break;
		}
		
		// gets a copy for display
		view = frame.clone();
		
		// converts frames to grayscale
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		
		// looks for chessboard in the frames
		std::vector<cv::Point2f> corners;
		cv::Size board = cv::Size(iCheckerBoardWidth, iCheckerBoardHeight);
		bool found= cv::findChessboardCorners(gray, board, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
			
		// checks if chessboard was found
		if (found && !bIsCalibrated)
		{
			cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

			// refines pixel coordinates for given 2d points
			cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1), criteria);

			// displays the detected corner points on the checkerboard
			cv::drawChessboardCorners(view, cv::Size(iCheckerBoardWidth, iCheckerBoardHeight), corners, found);
			
			cv::putText(view, "Press g to grab the frame", cv::Point(50, 50), 2, 1, cv::Scalar(0, 200, 0), 2);
		}
		
		if (!bIsCalibrated)
		{
			// adds frame number and displays it
			std::string sCounter = std::to_string(iCount) + "/" + std::to_string(iNbFrames);
			cv::putText(view, sCounter, cv::Point(50, 100), 2, 1, cv::Scalar(0, 0, 200), 2);
		}
		
		// displays the current frame
		cv::imshow("Frames", view);
			
		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] Esc key is pressed by user -> Shutting down!" << std::endl;
			return 3; 
		}
		
		// checks if 'g' was pressed to save the current frame
		if (key == 'g' && found) // if 'g' key is pressed, save the frame
		{
			std::cout << "[INFO] Checkerboard information is saved: " << iCount << "/" << iNbFrames << std::endl;
			
			// stores 3D and corresponding 2D points
			objpoints.push_back(objp);
			imgpoints.push_back(corners);
				
			// increments the counter
			iCount++;
			
			// creates a blink effect
			bitwise_not(view, view);
			cv::imshow("Frames", view);
			cv::waitKey(200);
		}
		
		// checks if 'u' was pressed to undistort the frame when calibration is done
		if (key == 'u' && bIsCalibrated) 
		{
			bIsUndistort = !bIsUndistort;
			view = frame.clone();
			
			if (bIsUndistort)
			{
				std::cout << "[INFO] Undistort frame"  << std::endl;
				undistort(frame, view, camMatrix, distCoeffs);
			}
			else
				std::cout << "[INFO] Original frame"  << std::endl;
			
			cv::imshow("Frames", view);
		}
		
		// checks if 's' was pressed to save the matrix intrinsic and distortion information to a yaml file
		if (key == 's' && bIsCalibrated)
		{
			// saves the calibration params in a yaml file
			std::cout << "[INFO] Save the result in a yaml file... ";
			bool isMonoCalibParamsSaved = writeMonoCameraParameters(sMonoCalibParamFilename, camMatrix, distCoeffs);
			
			// checks if the stereo camera parameters were successfully read
			if (!isMonoCalibParamsSaved)
			{
				std::cout << std::endl;
				std::cout << "\t[ERROR] Mono camera parameters could not be saved!" << std::endl;
				return 4;
			}
			std::cout << "OK!" << std::endl;
		}
			
		// performes calibration if number of frames is sufficient
		if (iCount >= iNbFrames && !bIsCalibrated)
		{
			view = frame.clone();
			cv::putText(view, "Calibration in progress...", cv::Point(50, 50), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::imshow("Frames", view);
			cv::waitKey(200);
			
			/*
			* Performing camera calibration by passing the value of known 3D points (objpoints)
			* and corresponding pixel coordinates of the detected corners (imgpoints)
			*/
			cv::Mat R, T;
			std::vector<cv::Point3f> newObjPoints;
			int iFixedPoint = -1;
				iFixedPoint = iCheckerBoardHeight - 1;
			int flag = 0;
				flag |= cv::CALIB_ZERO_TANGENT_DIST;
				flag |= cv::CALIB_FIX_ASPECT_RATIO;
			
			// calibrates camera
			std::cout << "[INFO] Calibrate the camera" << std::endl;
			double rms = cv::calibrateCameraRO(objpoints, imgpoints, gray.size(), iFixedPoint, camMatrix, distCoeffs, R, T, newObjPoints, flag | cv::CALIB_USE_LU);
			std::cout << "--> RMS reprojection error = " << rms << std::endl;	
			
			bIsCalibrated = true;
			
			view = frame.clone();
			cv::putText(view, "Calibration done", cv::Point(50, 50), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::putText(view, "Press u to visualize undistort frame", cv::Point(50, 100), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::putText(view, "Press s to save the intrinsic/distortion params", cv::Point(50, 150), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::imshow("Frames", view);
			cv::waitKey(-1);
		}
	}
	
	// releases video stream
	cam.release();
	
	// destroys all windows
	cv::destroyAllWindows();
	
	return 0;
}
	
	
	
	
	
	
	