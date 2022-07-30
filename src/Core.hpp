#pragma once

#include <Eigen/Core>

#include "Types.hpp"

// Image coordinate
// --------------------- col (x-axis) --------------------
// |
// |
// row (y-axis)
// |
// |

namespace alcc {
	// Generate the discontinuity cloud
	// The "intensity" field in the returned cloud is used for storing discontinuity value
	PtCloudXYZI_Type::Ptr GenDiscontinuityCloud(const PtCloudXYZI_Type &cloud);

	void CloudDiscontinuityFilter(PtCloudXYZI_Type& cloud, float discontinuity_lower_lim);

	void GenEdgeImage(const Img_Type &input, Img_Type &out);
	void GetPixelNeighbors(const Img_Type &img, const cv::Point2i &pixel, std::vector<int> &neighbor_vals);
	
	// Get the max absolute difference value between x and values in vec
	int GetMaxAbsDiffVal(int x, const std::vector<int> &vec);

	void InverseDistTransform(const Img_Type& edge_img, Img_Type& out);
	float ComputeMaxTermInInverseTransform(const Img_Type &edge_img, const cv::Point2i &pixel);
	
	float EuclideanDistanceToOrigin(const PointXYZI_Type &point);

	float SingelFrameCost(const Img_Type& img, const PtCloudXYZI_Type &cloud, const Eigen::Isometry3f &T_cl, const cv::Mat &intri);

	void PtCloudXYZIToCvPoint3f(const PtCloudXYZI_Type& cloud, std::vector<cv::Point3f>& result_pts, std::vector<float>& result_intensity);

	float GetSubPixelValBilinear(const cv::Mat& img, const cv::Point2f& pixel);
}