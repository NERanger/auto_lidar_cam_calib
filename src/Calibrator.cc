#include <mutex>
#include <random>

#include <glog/logging.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

#include "progressbar.hpp"

#include "Calibrator.hpp"
#include "Core.hpp"
#include "StopWatch.hpp"

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

void Calibrator::CalibrationTrack(const Eigen::Isometry3f& init, Eigen::Isometry3f& result, int iter_num) {
	static constexpr int grid_step_num = 2;
	static constexpr float grid_rot_step = 0.3f;
	static constexpr float grid_trans_step = 0.03f;

	std::vector<Eigen::Isometry3f> grid;
	GenGridIsometry3f(grid, init, grid_step_num, grid_rot_step, grid_trans_step);
	grid.push_back(init);
	LOG(INFO) << "Grid size: " << grid.size();

	ProcessDataFrames();

	StopWatch sw;
	for (int iter = 0; iter < iter_num; ++iter) {
		LOG(INFO) << "Iteration: " << iter + 1 << " out of " << iter_num;

		// Single thread implementation
		std::vector<float> costs(grid.size());
		for (size_t i = 0; i < grid.size(); ++i) {
			costs[i] = SingleExtriCost(grid[i]);
		}

		int max_index = std::max_element(costs.begin(), costs.end()) - costs.begin();
		result = grid[max_index];
	}

	LOG(INFO) << "Track time: " << sw.GetTimeElapse();

}

float Calibrator::MiscalibrationDetection(const Eigen::Isometry3f& T_cl) {
	std::vector<Eigen::Isometry3f> grid;
	GenGridIsometry3f(grid, T_cl, 1, 2.0f, 0.2f);
	grid.push_back(T_cl);

	LOG(INFO) << "Grid size: " << grid.size();

	ProcessDataFrames();

	std::vector<float> costs(grid.size());

#if 0
	cv::parallel_for_(cv::Range(0, grid.size()),
		[&grid, &costs, this](const cv::Range& range) {
			for (int r = range.start; r < range.end; ++r) {
				float cost = SingleExtriCost(grid[r]);
				costs[r] = cost;
			}
		});
#endif 

#if 1 
	// Single thread implementation
	for (size_t i = 0; i < grid.size(); ++i) {
		costs[i] = SingleExtriCost(grid[i]);
	}
#endif

	float center_cost = costs.back();
	int worse_num = 0;
	for (const float &c : costs) {
		if (center_cost > c) {
			worse_num += 1;
		}
	}

	return static_cast<float>(worse_num) / static_cast<float>(costs.size() - 1);
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
	static constexpr float discontinuity_lower_lim = 0.5f;

	if (!processed_frames_d_flag_) {
		return;
	}

	LOG(INFO) << "Data processing start, frame number: " << frames_.size();

	StopWatch sw;

	processed_frames_.clear();
	processed_frames_.resize(frames_.size());

#if 0
	// Multithread implementation
	cv::parallel_for_(cv::Range(0, frames_.size()),
		[this](const cv::Range& range) {
			for (int r = range.start; r < range.end; ++r) {
				const DataFrame_Type& raw_frame = frames_.at(r);

				cv::Mat edge_img, inv_trans;
				alcc::GenEdgeImage(raw_frame.img, edge_img);
				alcc::InverseDistTransform(edge_img, inv_trans);

				PtCloudXYZI_Type::Ptr discon_cloud = GenDiscontinuityCloud(*raw_frame.cloud);
				CloudDiscontinuityFilter(*discon_cloud, discontinuity_lower_lim);

				processed_frames_[r].cloud = discon_cloud;
				processed_frames_[r].img = inv_trans;
			}
		});
#endif

#if 1
	// Single thread implementation
	progressbar bar(frames_.size());
	for (size_t i = 0; i < frames_.size(); ++i) {
		const DataFrame_Type& f = frames_.at(i);

		cv::Mat edge_img, inv_trans;
		alcc::GenEdgeImage(f.img, edge_img);
		alcc::InverseDistTransform(edge_img, inv_trans);

		PtCloudXYZI_Type::Ptr discon_cloud = GenDiscontinuityCloud(*f.cloud);
		CloudDiscontinuityFilter(*discon_cloud, discontinuity_lower_lim);

		processed_frames_[i] = DataFrame_Type{ discon_cloud, inv_trans };
		
		bar.update();
	}
	std::cout << std::endl;
#endif

	processed_frames_d_flag_ = false;

	LOG(INFO) << "Processing time: " << sw.GetTimeElapse();
}