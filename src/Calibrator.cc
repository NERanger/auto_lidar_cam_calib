#include <mutex>
#include <random>

#include <glog/logging.h>

#include <opencv2/core/utility.hpp>

#include "progressbar.hpp"

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

	processed_frames_d_flag_ = true;
}

void Calibrator::CalibrationTrack(const Eigen::Isometry3f& init, Eigen::Isometry3f& result) {
	for (const DataFrame_Type &f : frames_) {
		SingelFrameCost(f.img, *f.cloud, init, intri_.AsCvMat());
	}
}

float Calibrator::MiscalibrationDetection(const Eigen::Isometry3f& T_cl) {
	std::vector<Eigen::Isometry3f> grid;
	GenGridIsometry3f(grid, T_cl, 1, 2.0f, 0.2f);

	grid.push_back(T_cl);

	std::vector<float> costs;
	costs.reserve(grid.size());

	LOG(INFO) << "Processing data frames...";
	ProcessDataFrames();
	LOG(INFO) << "Processing data frames... Done";

	std::mutex mutex;
	progressbar bar(grid.size());
	cv::parallel_for_(cv::Range(0, grid.size()),
		[&](const cv::Range& range) {
			for (int r = range.start; r < range.end; ++r) {
				float cost = SingleExtriCost(grid.at(r));

				std::lock_guard<std::mutex> lck(mutex);
				costs.push_back(cost);
				bar.update();
			}
		});

#if 0 
	// Single thread implementation
	// progressbar bar(sampled_grid.size());
	for (const Eigen::Isometry3f& extri : sampled_grid) {
		// bar.update();
		costs.push_back(SingleExtriCost(extri));
	}
#endif

	std::cout << std::endl;

	float center_cost = costs.back();
	int worse_num = 0;
	for (const float &c : costs) {
		if (center_cost < c) {
			worse_num += 1;
		}
	}

	return static_cast<float>(worse_num) / static_cast<float>(costs.size());
}

float Calibrator::SingleExtriCost(const Eigen::Isometry3f& T_cl) {
	cv::Mat intri_mat = intri_.AsCvMat();
	float total_cost = 0.0f;
	for (const DataFrame_Type& f : processed_frames_) {
		total_cost += SingelFrameCost(f.img, *f.cloud, T_cl, intri_mat);
	}

	return total_cost;
}

void Calibrator::ProcessDataFrames() {
	static constexpr float discontinuity_lower_lim = 0.3f;

	if (!processed_frames_d_flag_) {
		return;
	}

	processed_frames_.reserve(frames_.size());
	for (DataFrame_Type& f : frames_) {
		cv::Mat edge_img, inv_trans;
		alcc::GenEdgeImage(f.img, edge_img);
		alcc::InverseDistTransform(edge_img, inv_trans);

		PtCloudXYZI_Type::Ptr discon_cloud = GenDiscontinuityCloud(*f.cloud);
		CloudDiscontinuityFilter(*discon_cloud, discontinuity_lower_lim);

		processed_frames_.push_back(DataFrame_Type{ discon_cloud, inv_trans });
	}

	processed_frames_d_flag_ = false;
}