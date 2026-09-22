#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <iostream>

#include "opencv2/opencv.hpp"

float deg2rad(float angle);

float rad2deg(float angle);

std::vector<float> computeForwardKinematics(float q1, float q2, float L1, float L2);

std::vector<float> computeInverseKinematics(float x, float y, float L1, float L2);

std::vector<float> computeDifferentialKinematics(float q1, float q2, float L1, float L2);

int computeJacobianMatrixRank(std::vector<float> vJacobianMatrix, float threshold);

cv::Mat  computeInverseJacobianMatrix(std::vector<float> vJacobianMatrix);
