#pragma once

#include <string>
#include <filesystem>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/mat.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lvx {
	struct Intrinsic {
		float fx;
		float fy;
		float cx;
		float cy;

		cv::Mat AsCvMat() const;
	};

	struct Frame {
		cv::Mat img;
		pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud;
		Eigen::Isometry3f T_wl; // Transform from current lidar frame to world frame (first frame)
	};

	class LivoxLoader {
	public:
		LivoxLoader(const std::string& root);

		inline size_t Size() const { return size_; }
		inline Intrinsic GetIntrinsic() const { return intri_; }
		inline Eigen::Isometry3f GetExtrinsic() const { return T_cl_; }

		Frame operator[](size_t i) const;

	private:
		std::filesystem::path root_;

		std::filesystem::path img_dir_;
		std::filesystem::path lidar_dir_;
	
		std::filesystem::path gt_file_;

		Intrinsic intri_ = { 957.994f, 955.328f, 790.335f, 250.6631f};
		std::vector<float> distort_coeff = { -0.12f, 0.1162f, 0.0f, 0.0f };

		Eigen::Isometry3f T_cl_; // From lidar to camera

		size_t size_;

		std::vector<Eigen::Isometry3f> T_wl_;

		size_t LivoxLoader::GetFileNumInDir(const std::filesystem::path& p);
		void LoadGtPoses();
		cv::Mat LoadImg(const std::string &path) const;
		pcl::PointCloud<pcl::PointXYZI>::Ptr LoadCloud(const std::string &path) const;
	};
}