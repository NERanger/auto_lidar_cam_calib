#include <pcl/common/io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/calib3d.hpp>

#include <glog/logging.h>

#include "Core.hpp"
#include "Constants.hpp"

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
		
		result->at(i).intensity = std::powf(*max_ele, gamma);
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

	out = cv::Mat::zeros(edge_img.size(), CV_32FC1);

#if 1
	// Parallel implementation
	cv::parallel_for_(cv::Range(0, out.rows * out.cols),
		[&](const cv::Range& range) {
			for (int r = range.start; r < range.end; r++) {
				int row = r / out.cols;
				int col = r % out.cols;

				const uchar& edge_val = edge_img.at<uchar>(row, col);

				out.ptr<float>(row)[col] = alpha * edge_val + (1.0f - alpha) * ComputeMaxTermInInverseTransform(edge_img, cv::Point2i(col, row));
			}
		});
#endif

#if 0
	// Single thread implementation
	for (int col = 0; col < out.cols; ++col) {
		for (int row = 0; row < out.rows; ++row) {
			const uchar& edge_val = edge_img.at<uchar>(row, col);

			float& out_val = out.at<float>(row, col);
			out_val = alpha * edge_val + (1.0f - alpha) * ComputeMaxTermInInverseTransform(edge_img, cv::Point2i(col, row));
		}
	}
#endif
}

float alcc::ComputeMaxTermInInverseTransform(const Img_Type& edge_img, const cv::Point2i& pixel) {
	static constexpr float gamma = 0.98f;

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

float alcc::SingelFrameCost(const Img_Type& edge_img, const PtCloudXYZI_Type& discon_cloud, const Eigen::Isometry3f& T_cl, const cv::Mat& intri) {
	// Edge imamge generation
	// cv::Mat edge_img;
	// alcc::GenEdgeImage(img, edge_img);
	// cv::Mat edge_img_transed;
	// alcc::InverseDistTransform(edge_img, edge_img_transed);

	// Discontinuity point cloud generation
	// PtCloudXYZI_Type::Ptr discon_cloud = GenDiscontinuityCloud(cloud);
	// CloudDiscontinuityFilter(*discon_cloud, discontinuity_lower_lim);

	// Transform to camera coordinate
	PtCloudXYZI_Type::Ptr cloud_cam(new PtCloudXYZI_Type);
	pcl::transformPointCloud(discon_cloud, *cloud_cam, T_cl.matrix());

	std::vector<cv::Point3f> cvpts;
	std::vector<float> discon_vals;
	PtCloudXYZIToCvPoint3f(*cloud_cam, cvpts, discon_vals);

	std::vector<cv::Point2f> pixels;
	pixels.reserve(cvpts.size());
	cv::projectPoints(cvpts, Constants::UnitRotationVec(), cv::Mat::zeros(3, 1, CV_32FC1),
		intri, cv::Mat(), pixels);

	RemovePixelOutSideImg(pixels, discon_vals, edge_img.rows, edge_img.cols);

	float total_cost = 0.0f;
	for (size_t i = 0; i < pixels.size(); ++i) {
		total_cost += discon_vals[i] * GetSubPixelValBilinear(edge_img, pixels[i]);
	}

	return total_cost;

}

void alcc::PtCloudXYZIToCvPoint3f(const PtCloudXYZI_Type& cloud, std::vector<cv::Point3f>& result_pts, std::vector<float>& result_intensity) {
	result_pts.clear();
	result_pts.reserve(cloud.size());
	result_intensity.clear();
	result_intensity.reserve(cloud.size());

	for (const PointXYZI_Type &pt : cloud.points) {
		if (pt.z < 0.0f) { // Remove points with negative z value in camera coordinate
			continue;
		}
		result_pts.push_back(cv::Point3f(pt.x, pt.y, pt.z));
		result_intensity.push_back(pt.intensity);
	}
}

float alcc::GetSubPixelValBilinear(const cv::Mat& img, const cv::Point2f& pixel) {
	// CHECK_EQ(img.type(), CV_32FC1);

	int x = static_cast<int>(pixel.x);
	int y = static_cast<int>(pixel.y);

	int x0 = cv::borderInterpolate(x, img.cols, cv::BORDER_REFLECT_101);
	int x1 = cv::borderInterpolate(x + 1, img.cols, cv::BORDER_REFLECT_101);
	int y0 = cv::borderInterpolate(y, img.rows, cv::BORDER_REFLECT_101);
	int y1 = cv::borderInterpolate(y + 1, img.rows, cv::BORDER_REFLECT_101);

	float a = pixel.x - static_cast<float>(x);
	float c = pixel.y - static_cast<float>(y);

	float val_00 = static_cast<float>(img.at<float>(y0, x0));
	float val_01 = static_cast<float>(img.at<float>(y1, x0));
	float val_10 = static_cast<float>(img.at<float>(y0, x1));
	float val_11 = static_cast<float>(img.at<float>(y1, x1));

	return (val_00 * (1.f - a) + val_10 * a) * (1.f - c) + (val_01 * (1.f - a) + val_11 * a) * c;
}

void alcc::GenGridIsometry3f(std::vector<Eigen::Isometry3f>& grid, const Eigen::Isometry3f& center, int step_num, float rot_step, float trans_step) {
	static constexpr float kDim = 6;

	// step = 1 -> [-1, 0, 1]
	const int num_per_dimension = 1 + 2 * step_num;
	const int start_num = 0 - step_num;
	const int grid_size = std::pow(num_per_dimension, kDim);

	grid.clear();
	grid.reserve(grid_size);

	std::vector<int> loc(num_per_dimension);
	std::iota(loc.begin(), loc.end(), start_num);

	Eigen::Vector3f init_trans(center.translation());
	Eigen::Vector3f init_rot(center.rotation().eulerAngles(0, 1, 2));

	float rot_step_size_rad = rot_step * Constants::Deg2Rad();

	for (int tran_x = 0; tran_x < loc.size(); ++tran_x) {
		for (int tran_y = 0; tran_y < loc.size(); ++tran_y) {
			for (int tran_z = 0; tran_z < loc.size(); ++tran_z) {
				for (int rot_x = 0; rot_x < loc.size(); ++rot_x) {
					for (int rot_y = 0; rot_y < loc.size(); ++rot_y) {
						for (int rot_z = 0; rot_z < loc.size(); ++rot_z) {
							// If all zero, equal to center
							if (loc[tran_x] == loc[tran_y] && loc[tran_x] == loc[tran_z] &&
								loc[tran_x] == loc[rot_x] && loc[tran_x] == loc[rot_y] &&
								loc[tran_x] == loc[rot_z] && loc[tran_x] == 0) {
								continue;
							}
							Eigen::Vector3f trans(static_cast<float>(init_trans.x() + loc[tran_x] * trans_step),
								static_cast<float>(init_trans.y() + loc[tran_y] * trans_step),
								static_cast<float>(init_trans.z() + loc[tran_z] * trans_step));
							Eigen::Vector3f rot(static_cast<float>(init_rot.x() + loc[rot_x] * rot_step_size_rad),
								static_cast<float>(init_rot.y() + loc[rot_y] * rot_step_size_rad),
								static_cast<float>(init_rot.z() + loc[rot_z] * rot_step_size_rad));

							Eigen::Isometry3f iso(EulerToQuat(rot.x(), rot.y(), rot.z()));
							iso.pretranslate(trans);

							grid.push_back(iso);
						}
					}
				}
			}
		}
	}
}

Eigen::Quaternionf alcc::EulerToQuat(float x, float y, float z) {
	using Eigen::Quaternionf;
	using Eigen::AngleAxisf;
	using Eigen::Vector3f;

	Quaternionf q;
	q = AngleAxisf(x, Vector3f::UnitX())
		* AngleAxisf(y, Vector3f::UnitY())
		* AngleAxisf(z, Vector3f::UnitZ());

	return q;
}

void alcc::RemovePixelOutSideImg(std::vector<cv::Point2f>& pixels, std::vector<float> &discon_vals, int height, int width) {
	CHECK_EQ(pixels.size(), discon_vals.size());
	
	std::vector<cv::Point2f> pixels_res;
	std::vector<float> discon_res;

	int cnt = 0;
	for (size_t i = 0; i < pixels.size(); ++i) {
		const cv::Point2f& pixel = pixels.at(i);
		if (pixel.x < 0.0f || pixel.x > width || pixel.y < 0.0f || pixel.y > height) {
			continue;
		}

		pixels_res.push_back(pixel);
		discon_res.push_back(discon_vals.at(i));
	}

	pixels = pixels_res;
	discon_vals = discon_res;
}