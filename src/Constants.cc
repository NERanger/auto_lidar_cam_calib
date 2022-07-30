#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/calib3d.hpp>

#include "Constants.hpp"

using alcc::Constants;

const cv::Mat& Constants::UnitRotationVec() {
	static cv::Mat rot_vec;

	if (rot_vec.empty()) {
		cv::Rodrigues(cv::Mat::eye(3, 3, CV_32FC1), rot_vec);
	}

	return rot_vec;
}

const float& Constants::Deg2Rad() {
	static float deg2rad = -1.0f;

	if (deg2rad < 0.0f) {
		deg2rad = M_PI / 180.0f;
	}

	return deg2rad;
}