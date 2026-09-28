#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define CAM_PARAMS_FILENAME "./data/camera_calibration_params.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params_data.xml"

#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5
#define AREA_THRESOLD 1000

using namespace cv;
using namespace std;


// ==============================
// READ CAMERA PARAMETERS
// ==============================

bool readCameraParameters(
    std::string filename,
    cv::Mat &camMatrix,
    cv::Mat &distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout
            << "[ERROR] Could not open camera parameter file: "
            << filename
            << std::endl;

        return false;
    }

    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    return true;
}


// ==============================
// READ COLOR PARAMETERS
// ==============================

bool readColorParameters(
    std::string filename,
    int &iLowH,
    int &iHighH,
    int &iLowS,
    int &iHighS,
    int &iLowV,
    int &iHighV)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout
            << "[ERROR] Could not open color parameter file: "
            << filename
            << std::endl;

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


// ==============================
// MAIN
// ==============================

int main(int argc, char **argv)
{
    std::string sCameraParamFilename = CAM_PARAMS_FILENAME;
    std::string sColorParamFilename = COLOR_PARAMS_FILENAME;

    float fFPS = FPS;

    int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
    int iAreaThresold = AREA_THRESOLD;


    // ==============================
    // COMMAND-LINE PARAMETERS
    // ==============================

    int opt;

    while ((opt = getopt(argc, argv, ":c:f:s:a:i:")) != -1)
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
                if (
                    optopt == 'c' ||
                    optopt == 'f' ||
                    optopt == 's' ||
                    optopt == 'a' ||
                    optopt == 'i')
                {
                    fprintf(
                        stderr,
                        "Option -%c requires an argument.\n",
                        optopt);
                }
                else
                {
                    fprintf(
                        stderr,
                        "Unknown option.\n");
                }

                return 1;

            default:
                abort();
        }
    }


    // ==============================
    // LOAD HSV COLOR PARAMETERS
    // ==============================

    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;

    bool isColorParamsSet =
        readColorParameters(
            sColorParamFilename,
            iLowH,
            iHighH,
            iLowS,
            iHighS,
            iLowV,
            iHighV);

    if (!isColorParamsSet)
    {
        std::cout
            << "[ERROR] Color parameters could not be loaded!"
            << std::endl;

        return -1;
    }


    // ==============================
    // LOAD CAMERA CALIBRATION
    // ==============================

    bool bIsImageUndistorted = true;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    bool isCamParamsSet =
        readCameraParameters(
            sCameraParamFilename,
            cameraMatrix,
            distCoeffs);

    if (!isCamParamsSet)
    {
        std::cout
            << "[WARNING] Camera calibration could not be loaded!"
            << std::endl;
    }


    // ==============================
    // OPEN CAMERA
    // ==============================

    VideoCapture cap(0, cv::CAP_V4L2);

    if (!cap.isOpened())
    {
        std::cout
            << "[ERROR] Could not open camera!"
            << std::endl;

        return -1;
    }


    // ==============================
    // INITIAL BALL TRACKING
    // ==============================

    int iLastX = -1;
    int iLastY = -1;

    Mat imgTmp;

    cap.read(imgTmp);

    Mat imgLines =
        Mat::zeros(
            imgTmp.size(),
            CV_8UC3);


    // ==============================
    // ARUCO DICTIONARY
    // ==============================

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_4X4_50);


    // ==============================
    // STORED HOMOGRAPHY
    // IMPORTANT:
    // This is OUTSIDE the while loop.
    // ==============================

    cv::Mat homography;

    bool homographyReady = false;


    // ==============================
    // MAIN LOOP
    // ==============================

    while (true)
    {
        cv::Mat imgOriginal;

        bool bSuccess =
            cap.read(imgOriginal);

        if (!bSuccess)
        {
            std::cout
                << "[WARNING] Could not read a frame."
                << std::endl;

            break;
        }


        // ==============================
        // UNDISTORT IMAGE
        // ==============================

        if (
            bIsImageUndistorted &&
            isCamParamsSet)
        {
            cv::Mat temp =
                imgOriginal.clone();

            cv::undistort(
                temp,
                imgOriginal,
                cameraMatrix,
                distCoeffs);
        }


        // ==============================
        // ARUCO DETECTION
        // ==============================

        std::vector<int> markerIds;

        std::vector<
            std::vector<cv::Point2f>
        > markerCorners;

        cv::aruco::detectMarkers(
            imgOriginal,
            dictionary,
            markerCorners,
            markerIds);


        // ==============================
        // FIELD CALIBRATION
        //
        // Only calibrate if not already done.
        // Once done, keep homography forever
        // during this execution.
        // ==============================

        if (!homographyReady)
        {
            std::vector<cv::Point2f> imagePoints;
            std::vector<cv::Point2f> worldPoints;

            for (
                size_t i = 0;
                i < markerIds.size();
                i++)
            {
                cv::Point2f center(
                    0.0f,
                    0.0f);

                for (
                    int j = 0;
                    j < 4;
                    j++)
                {
                    center +=
                        markerCorners[i][j];
                }

                center *= 0.25f;


                // ID 8 = top left
                if (markerIds[i] == 8)
                {
                    imagePoints.push_back(center);

                    worldPoints.push_back(
                        cv::Point2f(
                            -21.4f,
                            9.15f));
                }


                // ID 7 = top right
                else if (markerIds[i] == 7)
                {
                    imagePoints.push_back(center);

                    worldPoints.push_back(
                        cv::Point2f(
                            21.4f,
                            9.15f));
                }


                // ID 6 = bottom left
                else if (markerIds[i] == 6)
                {
                    imagePoints.push_back(center);

                    worldPoints.push_back(
                        cv::Point2f(
                            -21.4f,
                            -9.15f));
                }


                // ID 9 = bottom right
                else if (markerIds[i] == 9)
                {
                    imagePoints.push_back(center);

                    worldPoints.push_back(
                        cv::Point2f(
                            21.4f,
                            -9.15f));
                }
            }


            if (imagePoints.size() == 4)
            {
                cv::Mat newHomography =
                    cv::findHomography(
                        imagePoints,
                        worldPoints);

                if (!newHomography.empty())
                {
                    homography =
                        newHomography.clone();

                    homographyReady =
                        true;

                    std::cout
                        << "[INFO] Field calibration complete!"
                        << std::endl;
                }
            }
            else
            {
                std::cout
                    << "[INFO] Waiting for all 4 ArUco markers..."
                    << std::endl;
            }
        }


        // ==============================
        // HSV CONVERSION
        // ==============================

        cv::Mat imgHSV;

        cvtColor(
            imgOriginal,
            imgHSV,
            cv::COLOR_BGR2HSV);


        // ==============================
        // COLOR THRESHOLD
        // ==============================

        cv::Mat imgThresholded;

        inRange(
            imgHSV,
            cv::Scalar(
                iLowH,
                iLowS,
                iLowV),
            cv::Scalar(
                iHighH,
                iHighS,
                iHighV),
            imgThresholded);


        // ==============================
        // MORPHOLOGICAL OPENING
        // ==============================

        cv::erode(
            imgThresholded,
            imgThresholded,
            getStructuringElement(
                MORPH_ELLIPSE,
                Size(
                    iStructuralElementSize,
                    iStructuralElementSize)));

        cv::dilate(
            imgThresholded,
            imgThresholded,
            getStructuringElement(
                MORPH_ELLIPSE,
                Size(
                    iStructuralElementSize,
                    iStructuralElementSize)));


        // ==============================
        // MORPHOLOGICAL CLOSING
        // ==============================

        cv::dilate(
            imgThresholded,
            imgThresholded,
            getStructuringElement(
                MORPH_ELLIPSE,
                Size(
                    iStructuralElementSize,
                    iStructuralElementSize)));

        cv::erode(
            imgThresholded,
            imgThresholded,
            getStructuringElement(
                MORPH_ELLIPSE,
                Size(
                    iStructuralElementSize,
                    iStructuralElementSize)));


        // ==============================
        // BALL CENTER USING MOMENTS
        // ==============================

        Moments oMoments =
            moments(
                imgThresholded);

        double dM01 =
            oMoments.m01;

        double dM10 =
            oMoments.m10;

        double dArea =
            oMoments.m00;


        if (dArea > iAreaThresold)
        {
            int posX =
                dM10 / dArea;

            int posY =
                dM01 / dArea;


            // ==============================
            // PIXEL POSITION
            // ==============================

            std::cout
                << "Ball position: X = "
                << posX
                << " px, Y = "
                << posY
                << " px"
                << std::endl;


            // ==============================
            // PIXEL -> WORLD CM
            // ==============================

            if (homographyReady)
            {
                std::vector<cv::Point2f>
                    ballPixel;

                std::vector<cv::Point2f>
                    ballWorld;

                ballPixel.push_back(
                    cv::Point2f(
                        static_cast<float>(posX),
                        static_cast<float>(posY)));

                cv::perspectiveTransform(
                    ballPixel,
                    ballWorld,
                    homography);


                if (!ballWorld.empty())
                {
                    // Small vertical correction
                    // measured when the ball was at
                    // the physical center.
                    ballWorld[0].y += 1.31f;

                    std::cout
                        << "Ball world position: X = "
                        << ballWorld[0].x
                        << " cm, Y = "
                        << ballWorld[0].y
                        << " cm"
                        << std::endl;
                }
            }


            // ==============================
            // DRAW TRACKING LINE
            // ==============================

            if (
                iLastX >= 0 &&
                iLastY >= 0 &&
                posX >= 0 &&
                posY >= 0)
            {
                line(
                    imgLines,
                    Point(
                        posX,
                        posY),
                    Point(
                        iLastX,
                        iLastY),
                    Scalar(
                        0,
                        0,
                        255),
                    2);
            }


            iLastX =
                posX;

            iLastY =
                posY;
        }


        // ==============================
        // DISPLAY THRESHOLD IMAGE
        // ==============================

        imshow(
            "Thresholded Image",
            imgThresholded);


        // ==============================
        // DRAW BALL TRAJECTORY
        // ==============================

        imgOriginal =
            imgOriginal +
            imgLines;


        // ==============================
        // DRAW ARUCO MARKERS
        //
        // Done AFTER ball detection
        // so it does not affect HSV.
        // ==============================

        if (!markerIds.empty())
        {
            cv::aruco::drawDetectedMarkers(
                imgOriginal,
                markerCorners,
                markerIds);
        }


        // ==============================
        // DISPLAY ORIGINAL IMAGE
        // ==============================

        imshow(
            "Original",
            imgOriginal);


        // ==============================
        // KEYBOARD
        // ==============================

        char key =
            (char)cv::waitKey(
                1000.0 / fFPS);


        // ESC = quit
        if (key == 27)
        {
            std::cout
                << "[INFO] Shutting down!"
                << std::endl;

            break;
        }


        // U = toggle undistortion
        if (key == 'u')
        {
            bIsImageUndistorted =
                !bIsImageUndistorted;

            std::cout
                << "[INFO] Image undistorted: "
                << bIsImageUndistorted
                << std::endl;
        }
    }


    return 0;
}
