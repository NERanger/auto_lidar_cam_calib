#pragma once

#include <string>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/mat.hpp>

#include "mem2disk/Mem2Disk.hpp"

#include "Types.hpp"

namespace alcc {
	class FrameNumVec6DataBag : public m2d::IDataObj {
	public:
		using Ptr = std::shared_ptr<FrameNumVec6DataBag>;

		FrameNumVec6DataBag(const std::string& name) : name_(name) {}

		void AddData(int frame_idx, const Vector6f &vec);

		void DumpToDisk() override;
	private:
		std::string name_;

		std::vector<std::pair<int, Vector6f>> data_;
	};

	Eigen::Matrix<float, 6, 1> GetIsometryErr(const Eigen::Isometry3f &iso_1, const Eigen::Isometry3f &iso_2);
	
	// R - rotation around x-axis
	// P - rotation around y-axis
	// Y - rotation around z-axis
	Eigen::Matrix<float, 6, 1> IsometryToXYZRPY(const Eigen::Isometry3f &iso);
}