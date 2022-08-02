#include <filesystem>
#include <fstream>

#include <glog/logging.h>

#include "Misc.hpp"
#include "Constants.hpp"

using alcc::FrameNumVec6DataBag;

void FrameNumVec6DataBag::AddData(int frame_idx, const Vector6f& vec) {
	data_.push_back(std::make_pair(frame_idx, vec));
}

void FrameNumVec6DataBag::DumpToDisk() {
	namespace sf = std::filesystem;
	
	std::string root = m2d::DataManager::GetRoot();
	CHECK(!root.empty());

	sf::path file_path(root);
	file_path = file_path / sf::path(name_ + ".txt");

	float rad2deg = Constants::Rad2Deg();

	std::ofstream fs(file_path.string());
	for (const auto& d : data_) {
		const Vector6f &vec = d.second;
		fs << d.first
			<< " " << vec(0) << " " << vec(1) << " " << vec(2)
			<< " " << vec(3) * rad2deg << " " << vec(4) * rad2deg << " " << vec(5) * rad2deg << std::endl;
	}

	fs.close();
}

Eigen::Matrix<float, 6, 1> alcc::GetIsometryErr(const Eigen::Isometry3f& iso_1, const Eigen::Isometry3f& iso_2) {
	using Vector6f = Eigen::Matrix<float, 6, 1>;

	// R: around x-axis
	// P: around y-axis
	// Y: around z-axis
	Vector6f iso_1_vec6 = IsometryToXYZRPY(iso_1);
	Vector6f iso_2_vec6 = IsometryToXYZRPY(iso_2);

	return iso_1_vec6 - iso_2_vec6;
}

Eigen::Matrix<float, 6, 1> alcc::IsometryToXYZRPY(const Eigen::Isometry3f& iso) {
	using Vector6f = Eigen::Matrix<float, 6, 1>;

	Vector6f xyzrpy;

	Eigen::Vector3f xyz = iso.translation();
	Eigen::Vector3f rpy = iso.rotation().eulerAngles(0, 1, 2);

	xyzrpy(0) = xyz(0);
	xyzrpy(1) = xyz(1);
	xyzrpy(2) = xyz(2);
	xyzrpy(3) = rpy(0);
	xyzrpy(4) = rpy(1);
	xyzrpy(5) = rpy(2);

	return xyzrpy;
}