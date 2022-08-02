#pragma once

#include <opencv2/core/mat.hpp>

namespace alcc {
	class Constants {
	public:
		Constants() = delete;

		static const cv::Mat& UnitRotationVec();
		static const float& Deg2Rad();
		static const float& Rad2Deg();
	};
}