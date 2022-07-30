#include "Types.hpp"

using alcc::CamIntrisic;

Eigen::Matrix3f CamIntrisic::AsMat() const {
	Eigen::Matrix3f mat;
	mat <<
		fx, 0.0f, cx,
		0.0f, fy, cy,
		0.0f, 0.0f, 1.0f;

	return mat;
}

cv::Mat CamIntrisic::AsCvMat() const {
	cv::Mat mat = cv::Mat::eye(3, 3, CV_32FC1);
	mat.at<float>(0, 0) = fx;
	mat.at<float>(1, 1) = fy;
	mat.at<float>(0, 2) = cx;
	mat.at<float>(1, 2) = cy;

	return mat;
}