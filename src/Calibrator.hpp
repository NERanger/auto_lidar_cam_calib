#pragma once

#include "Types.hpp"

namespace alcc {
	class Calibrator {
	public:
		Calibrator();

		inline void SetUseMask(bool use_mask) { use_mask_ = use_mask; }
		inline void SetMaxFrameNum(size_t num) { max_num_ = num; }
		inline void SetCameraIntrinsic(float fx, float fy, float cx, float cy) { intri_ = CamIntrisic{fx, fy, cx, cy}; }

		void AddDataFrame(const PtCloudXYZI_Type::Ptr &cloud, const Img_Type &img);

		// Extrinsic in T_cl
		void CalibrationTrack(const Eigen::Isometry3f &init, Eigen::Isometry3f& result, int iter_num);

		float MiscalibrationDetection(const Eigen::Isometry3f& T_cl);

	private:

		float SingleExtriCost(const Eigen::Isometry3f& T_cl);
		void ProcessDataFrames();

		std::vector<DataFrame_Type> frames_;
		size_t max_num_ = 1;

		std::vector<DataFrame_Type> processed_frames_;
		bool processed_frames_d_flag_ = true;

		bool use_mask_ = false;

		CamIntrisic intri_;
	};
}