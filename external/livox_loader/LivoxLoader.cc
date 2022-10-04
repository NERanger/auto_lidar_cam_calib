#include <fstream>

#include <boost/format.hpp>

#include <glog/logging.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/io/pcd_io.h>

#include "LivoxLoader.hpp"

using lvx::LivoxLoader;
using lvx::Intrinsic;
using lvx::Frame;

cv::Mat Intrinsic::AsCvMat() const {
	cv::Mat mat = cv::Mat::eye(3, 3, CV_32FC1);
	mat.at<float>(0, 0) = fx;
	mat.at<float>(1, 1) = fy;
	mat.at<float>(0, 2) = cx;
	mat.at<float>(1, 2) = cy;

	return mat;
}

LivoxLoader::LivoxLoader(const std::string& root) {
	namespace sf = std::filesystem;

	CHECK(sf::exists(root));

	root_ = sf::path(root);
	CHECK(sf::exists(root_));
	LOG(INFO) << "Livox dataset root: " << root_.string();

	img_dir_ = root_ / "image";
	CHECK(sf::exists(img_dir_));
	LOG(INFO) << "Image directory: " << img_dir_.string();

	lidar_dir_ = root_ / "livox";
	CHECK(sf::exists(lidar_dir_));
	LOG(INFO) << "LiDAR directory: " << lidar_dir_.string();

	gt_file_ = root_ / "groundtruth.txt";
	CHECK(sf::exists(gt_file_));
	LOG(INFO) << "GT file: " << gt_file_.string();

	size_t img_size = GetFileNumInDir(img_dir_);
	size_t lidar_size = GetFileNumInDir(lidar_dir_);
	CHECK_EQ(img_size, lidar_size);
	size_ = img_size;
	LOG(INFO) << "Total frame number: " << size_;

	LoadGtPoses();

	T_cl_.matrix() <<
		0.00554604f, -0.999971f, -0.00523653f, 0.0316362f,
		-0.000379382f, 0.00523451f, -0.999986f, 0.0380934f,
		0.999985f, 0.00554795f, -0.000350341f, 0.409066f,
		0.0f, 0.0f, 0.0f, 1.0f;

	LOG(INFO) << "Intrinsic: \n" << intri_.AsCvMat();
	LOG(INFO) << "Extrinsic: \n" << T_cl_.matrix();
}

// Reference: https://stackoverflow.com/questions/41304891/how-to-count-the-number-of-files-in-a-directory-using-standard
size_t LivoxLoader::GetFileNumInDir(const std::filesystem::path& p) {
	using std::filesystem::directory_iterator;
	return std::distance(directory_iterator(p), directory_iterator{});
}

void LivoxLoader::LoadGtPoses() {
	std::ifstream fin(gt_file_.string());
	for (int i = 0; i < size_; ++i) {
		float data[12];
		fin >> data[0] >> data[1] >> data[2] >> data[3] >>
			data[4] >> data[5] >> data[6] >> data[7] >>
			data[8] >> data[9] >> data[10] >> data[11];

		Eigen::Matrix4f m;
		m << data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7],
			data[8], data[9], data[10], data[11],
			0.0f, 0.0f, 0.0f, 1.0f;
		T_wl_.push_back(Eigen::Isometry3f(m));
	}
}

cv::Mat LivoxLoader::LoadImg(const std::string& path) const {
	cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
	cv::Mat undistort_img;
	cv::undistort(img, undistort_img, intri_.AsCvMat(), distort_coeff);
	return undistort_img;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LivoxLoader::LoadCloud(const std::string& path) const {
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile(path, *cloud);
	return cloud;
}

Frame LivoxLoader::operator[](size_t i) const {
	using boost::format;
	format fmt_lidar("%s/%06d.pcd");
	format fmt_img("%s/%06d.png");

	std::string ptcloud = (fmt_lidar % lidar_dir_.string() % i).str();
	std::string img = (fmt_img % img_dir_.string() % i).str();

	Frame f;
	f.img = LoadImg(img);
	f.ptcloud = LoadCloud(ptcloud);
	f.T_wl = T_wl_.at(i);

	return f;
}