#include <glog/logging.h>

#include "Calibrator.hpp"
#include "Core.hpp"

using alcc::Calibrator;

Calibrator::Calibrator() {
}

void Calibrator::AddDataFrame(const PtCloudXYZI_Type::Ptr& cloud, const Img_Type& img) {
	frames_.push_back(DataFrame_Type{cloud, img});
	if (frames_.size() > max_num_) {
		frames_.erase(frames_.begin());
	}
}

void Calibrator::CalibrationTrack(const Eigen::Isometry3f& init, Eigen::Isometry3f& result) {
	
}