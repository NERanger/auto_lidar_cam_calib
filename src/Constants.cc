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