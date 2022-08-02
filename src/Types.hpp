#pragma once

#include <opencv2/core/mat.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace alcc {
	using PointXYZI_Type = pcl::PointXYZI;
	using PtCloudXYZI_Type = pcl::PointCloud<pcl::PointXYZI>;
	using Img_Type = cv::Mat;
	using Vector6f = Eigen::Matrix<float, 6, 1>;

	struct DataFrame_Type {
		PtCloudXYZI_Type::Ptr cloud;
		cv::Mat img;
	};

	struct CamIntrisic {
		float fx = 0.0f;
		float fy = 0.0f;
		float cx = 0.0f;
		float cy = 0.0f;

		Eigen::Matrix3f AsMat() const;

		cv::Mat AsCvMat() const;
	};
}