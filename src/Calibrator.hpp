#pragma once

#include "Types.hpp"

namespace alcc {
	class Calibrator {
	public:
		Calibrator();

		inline void SetMaxFrameNum(size_t num) { max_num_ = num; }
		inline void SetCameraIntrinsic(float fx, float fy, float cx, float cy) { intri_ = CamIntrisic{fx, fy, cx, cy}; }

		void AddDataFrame(const PtCloudXYZI_Type::Ptr &cloud, const Img_Type &img);

		// Extrinsic in T_cl
		void CalibrationTrack(const Eigen::Isometry3f &init, Eigen::Isometry3f& result);

	private:

		std::vector<DataFrame_Type> frames_;
		size_t max_num_ = 1;

		CamIntrisic intri_;
	};
}