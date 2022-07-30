#pragma once

#include <opencv2/core/mat.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace alcc {
	using PointXYZI_Type = pcl::PointXYZI;
	using PtCloudXYZI_Type = pcl::PointCloud<pcl::PointXYZI>;
	using Img_Type = cv::Mat;

	struct DataFrame_Type {
		PtCloudXYZI_Type::Ptr cloud;
		cv::Mat img;
	};

	struct CamIntrisic {
		float fx = 0.0f;
		float fy = 0.0f;
		float cx = 0.0f;
		float cy = 0.0f;

		const Eigen::Matrix3f& AsMat() const;

		const cv::Mat& AsCvMat() const;
	};
}