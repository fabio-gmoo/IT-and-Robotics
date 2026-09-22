#include "Kinematics.h"


float deg2rad(float angle)
{
	return angle/180.0*M_PI;
}

float rad2deg(float angle)
{
	return angle*180.0/M_PI;
}

std::vector<float> computeForwardKinematics(float q1, float q2, float L1, float L2)
{
	float x = L1 * cos(q1) + L2 * cos(q1+q2);
	float y = L1 * sin(q1) + L2 * sin(q1+q2);
	std::cout << "[INFO] Forward Kinematics : (q1, q2)->(x, y) = (" << rad2deg(q1) << ", " << rad2deg(q2) << ")->(" << x << ", " << y << ")" << std::endl;
	std::vector<float> X;
	X.push_back(x);
	X.push_back(y);
	
	return X;
}

std::vector<float> computeInverseKinematics(float x, float y, float L1, float L2)
{
	std::vector<float> qi;
	
	float cos_q2 = (x*x+y*y-(L1*L1+L2*L2)) / (2.0 * L1 * L2);
	
	std::cout << "[INFO] cos_q2= " << cos_q2 << std::endl;
	
	if (cos_q2 >1 | cos_q2 <-1)
	{
		qi.push_back(0.0);
		std::cout << "[INFO] Inverse Kinematics: No solution!" << std::endl;
	} 
	else if (cos_q2 == 1)
	{
		qi.push_back(1.0);
		float q1 = atan2(y, x);
		float q2 = 0;
		std::cout << "[INFO] Inverse Kinematics: One solution: (x, y)->(q1, q2) = (" << x << ", " << y << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
	}
	else if (cos_q2 == -1)
	{
		qi.push_back(1.0);
		float q1 = atan2(y, x);
		float q2 = M_PI;
		std::cout << "[INFO] Inverse Kinematics: One solution: (x, y)->(q1, q2) = (" << x << ", " << y << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
	}
	else
	{
		qi.push_back(2.0);
		std::cout << "[INFO] Inverse Kinematics: Two solutions: "<< std::endl;
		
		float q2 = acos(cos_q2);
		float q1 = (float)(atan2(y, x) - atan2(L2*sin(q2), L1+L2*cos_q2));
		std::cout << "\t(x, y)->(q1, q2) = (" << x << ", " << y << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
		
		
		q2 = -acos(cos_q2);
		q1 = (float)(atan2(y, x) - atan2(L2*sin(q2), L1+L2*cos_q2));
		
		std::cout << "\t(x, y)->(q1, q2) = (" << x << ", " << y << ")->(" << rad2deg(q1) << ", " << rad2deg(q2) << ")" << std::endl;
		qi.push_back(q1);
		qi.push_back(q2);
	}
	
	return qi;
}

std::vector<float> computeDifferentialKinematics(float q1, float q2, float L1, float L2)
{
	std::vector<float> jacobian;
	
	float j11 = -L2*sin(q1+q2) - L1*sin(q1);
	float j12 = -L2*sin(q1+q2);
	float j21 = L2*cos(q1+q2) + L1*cos(q1); 
	float j22 = L2*cos(q1+q2);
	
	jacobian.push_back(j11);
	jacobian.push_back(j12);
	jacobian.push_back(j21);
	jacobian.push_back(j22);
	
	return jacobian;
}

int computeJacobianMatrixRank(std::vector<float> vJacobianMatrix, float threshold)
{
	int rank = -1;
	cv::Mat1f oJacobianMatrix(2, 2);
	
	if (vJacobianMatrix.size() == 4)
	{
		// Converts the Jacobian matrix from std::vector to cv::Mat
		oJacobianMatrix.at<float>(0, 0) = vJacobianMatrix[0];
		oJacobianMatrix.at<float>(0, 1) = vJacobianMatrix[1];
		oJacobianMatrix.at<float>(1, 0) = vJacobianMatrix[2];
		oJacobianMatrix.at<float>(1, 1) = vJacobianMatrix[3];
		std::cout << "=====Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(0,0) << ", " <<  oJacobianMatrix.at<float>(0,1)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(1,0) << ", " <<  oJacobianMatrix.at<float>(1,1)  << " ]" << std::endl;
		// Computes the determinant of the Jacobian matrix
		float determinant = abs(vJacobianMatrix[0] * vJacobianMatrix[3] - vJacobianMatrix[1]*vJacobianMatrix[2]);
		std::cout << "=====Determinant of the Jacobian matrix=====" << std::endl << determinant << std::endl;
		// Computes SVD
		cv::Mat1f w, u, vt;
		cv::SVD::compute(oJacobianMatrix, w, u, vt);
		// Finds non zero singular values
		cv::Mat1f nonZeroSingularValues = w/w.at<float>(0,0) > threshold;
		// Counts the number of non zero singular values
		rank = cv::countNonZero(nonZeroSingularValues);
		std::cout << "=====Rank of the Jacobian matrix=====" << std::endl << rank << " / " << oJacobianMatrix.rows << std::endl;
		// Determines the inverse of the Jacobian matrix
		cv::Mat oJacobianInverse =  oJacobianMatrix.inv(); 
		std::cout << "=====Inverse of the Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(0,0) << ", " <<  oJacobianInverse.at<float>(0,1)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(1,0) << ", " <<  oJacobianInverse.at<float>(1,1)  << " ]" << std::endl;
	}
	else
		std::cout << "[ERROR] Jacobian matrix has a size of "<< vJacobianMatrix.size() << " instead of 4" << std::endl;

	return rank;
}

cv::Mat  computeInverseJacobianMatrix(std::vector<float> vJacobianMatrix)
{
	cv::Mat1f oJacobianMatrix(2, 2);
	cv::Mat oJacobianInverse;
	
	if (vJacobianMatrix.size() == 4)
	{
		// Converts the Jacobian matrix from std::vector to cv::Mat
		oJacobianMatrix.at<float>(0, 0) = vJacobianMatrix[0];
		oJacobianMatrix.at<float>(0, 1) = vJacobianMatrix[1];
		oJacobianMatrix.at<float>(1, 0) = vJacobianMatrix[2];
		oJacobianMatrix.at<float>(1, 1) = vJacobianMatrix[3];
		std::cout << "=====Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(0,0) << ", " <<  oJacobianMatrix.at<float>(0,1)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianMatrix.at<float>(1,0) << ", " <<  oJacobianMatrix.at<float>(1,1)  << " ]" << std::endl;
		// Determines the inverse of the Jacobian matrix
		cv::invert(oJacobianMatrix, oJacobianInverse, cv::DECOMP_SVD);
		//oJacobianInverse =  oJacobianMatrix.inv(); 
		std::cout << "=====Inverse of the Jacobian Matrix=====" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(0,0) << ", " <<  oJacobianInverse.at<float>(0,1)  << " ]" << std::endl;
		std::cout << "[ " << oJacobianInverse.at<float>(1,0) << ", " <<  oJacobianInverse.at<float>(1,1)  << " ]" << std::endl;
	}
	else
		std::cout << "[ERROR] Jacobian matrix has a size of "<< vJacobianMatrix.size() << " instead of 4" << std::endl;

	return oJacobianInverse;
}