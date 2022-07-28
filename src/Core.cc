#include <pcl/common/io.h>
#include <pcl/filters/conditional_removal.h>

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <glog/logging.h>

#include "Core.hpp"

using alcc::PtCloudXYZI_Type;
using alcc::PointXYZI_Type;
using alcc::Img_Type;

PtCloudXYZI_Type::Ptr alcc::GenDiscontinuityCloud(const PtCloudXYZI_Type& cloud) {
	static constexpr float gamma = 0.5f;
	
	PtCloudXYZI_Type::Ptr result(new PtCloudXYZI_Type);
	pcl::copyPointCloud(cloud, *result);
	for (size_t i = 1; i < cloud.size() - 1; ++i) {
		std::array<float, 3> dist{0.0f, 0.0f, 0.0f};

		dist[0] = EuclideanDistanceToOrigin(cloud.at(i - 1)) - EuclideanDistanceToOrigin(cloud.at(i));
		dist[1] = EuclideanDistanceToOrigin(cloud.at(i + 1)) - EuclideanDistanceToOrigin(cloud.at(i));

		auto max_ele = std::max_element(dist.begin(), dist.end());
		
		result->at(i).intensity = std::powf(*max_ele, gamma);;
	}

	return result;
}

void alcc::CloudDiscontinuityFilter(PtCloudXYZI_Type& cloud, float discontinuity_lower_lim) {
	auto remove_it = std::remove_if(cloud.begin(), cloud.end(), 
		[discontinuity_lower_lim](const PointXYZI_Type &pt) {
			return pt.intensity < discontinuity_lower_lim;
		});

	cloud.erase(remove_it, cloud.end());
}

void alcc::GenEdgeImage(const Img_Type& input, Img_Type& out) {
	out = cv::Mat::zeros(input.size(), CV_8UC1);
	
	for (int col = 0; col < input.cols; ++col) {
		for (int row = 0; row < input.rows; ++row) {
			int pixel_val = static_cast<int>(input.at<uchar>(row, col));

			std::vector<int> neighbor_vals;
			GetPixelNeighbors(input, cv::Point2i(col, row), neighbor_vals);

			int max_abs_diff = GetMaxAbsDiffVal(pixel_val, neighbor_vals);
			out.at<uchar>(row, col) = static_cast<uchar>(max_abs_diff);
		}
	}
}
int alcc::GetMaxAbsDiffVal(int x, const std::vector<int>& vec){
	using Eigen::VectorXi;

	std::vector<int> copy(vec);

	VectorXi vector_x = VectorXi::Ones(vec.size()) * x;
	Eigen::Map<VectorXi> vector_vec(copy.data(), copy.size());

	return (vector_x - vector_vec).cwiseAbs().maxCoeff();
}

void alcc::GetPixelNeighbors(const Img_Type& img, const cv::Point2i& pixel, std::vector<int>& neighbor_vals) {
	std::vector<cv::Point2i> neighbor_points{
		cv::Point2i(pixel.x - 1, pixel.y - 1), cv::Point2i(pixel.x, pixel.y - 1), cv::Point2i(pixel.x + 1, pixel.y - 1),
		cv::Point2i(pixel.x - 1, pixel.y), cv::Point2i(pixel.x + 1, pixel.y),
		cv::Point2i(pixel.x - 1, pixel.y + 1), cv::Point2i(pixel.x, pixel.y + 1), cv::Point2i(pixel.x + 1, pixel.y + 1)
	};

	neighbor_vals.reserve(neighbor_points.size());
	for (const cv::Point2i &pt : neighbor_points) {
		uchar val = img.at<uchar>(cv::borderInterpolate(pt.y, img.rows, cv::BORDER_REPLICATE),
								  cv::borderInterpolate(pt.x, img.cols, cv::BORDER_REPLICATE));

		neighbor_vals.push_back(static_cast<int>(val));
	}
}

float alcc::EuclideanDistanceToOrigin(const PointXYZI_Type& point) {
	return Eigen::Vector3f(point.x, point.y, point.z).norm();
}

void alcc::InverseDistTransform(const Img_Type& edge_img, Img_Type& out) {
	static constexpr float alpha = 1.0f / 3.0f;

	out = cv::Mat::zeros(edge_img.size(), CV_8UC1);
	for (int col = 0; col < out.cols; ++col) {
		for (int row = 0; row < out.rows; ++row) {
			const uchar& edge_val = edge_img.at<uchar>(row, col);

			uchar& out_val = out.at<uchar>(row, col);
			out_val = alpha * edge_val + (1.0f - alpha) * ComputeMaxTermInInverseTransform(edge_img, cv::Point2i(col, row));

			LOG_EVERY_N(INFO, 1000) << col << " " << row;
		}
	}
	
}

float alcc::ComputeMaxTermInInverseTransform(const Img_Type& edge_img, const cv::Point2i& pixel) {
	static constexpr float gamma = 0.98f;
	static constexpr int win_size = 55;

	cv::Mat val_map = cv::Mat::zeros(edge_img.size(), CV_32FC1);
	for (int col = 0; col < edge_img.cols; ++col) {
		for (int row = 0; row < edge_img.rows; ++row) {
			const uchar& edge_val = edge_img.at<uchar>(row, col);
			
			Eigen::Vector2i pixel_diff(pixel.x - col, pixel.y - row);
			int max_l1_dist = pixel_diff.cwiseAbs().maxCoeff();

			float val = edge_val * powf(gamma, max_l1_dist);
			val_map.at<float>(row, col) = val;
		}
	}

	double max_val;
	cv::minMaxLoc(val_map, NULL, &max_val, NULL, NULL);
	
	return static_cast<float>(max_val);
}