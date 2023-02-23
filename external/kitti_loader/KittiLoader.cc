#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include "KittiLoader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::ifstream;
using std::vector;

using kitti::KittiLoader;
using kitti::Frame;
using kitti::Intrinsics;

KittiLoader::KittiLoader(const string &data_root, bool &if_success) : root_(data_root){
    using boost::filesystem::exists;

    lidar_path_ = root_ / ("velodyne");
    left_img_path_ = root_ / ("image_0");
    right_img_path_ = root_ / ("image_1");
    cali_file_path_ = root_ / ("calib.txt");
    gt_pose_path_ = root_ / ("groundtruth.txt");

    cout << "[KittiLoader] Expected data path:" << endl
         << "-- LiDAR data: " << lidar_path_.string() << endl
         << "-- Left camera image: " << left_img_path_.string() << endl
         << "-- Right camera image: " << right_img_path_.string() << endl
         << "-- Calibration file: " << cali_file_path_.string() << endl
         << "-- Groundtruth file: " << gt_pose_path_.string() << endl;

    if(!(exists(lidar_path_) && exists(left_img_path_) && exists(right_img_path_) && exists(cali_file_path_))){
        cerr << "[KittiLoader] Dataset not complete, check if the expected data path exists." << endl;
        if_success = false;
    }

    LoadCaliData(cali_file_path_.string());

    cout << "[KittiLoader] Left camera intrinsics:" << endl
        << "-- fx: " << left_cam_intrinsics_.fx << endl
        << "-- fy: " << left_cam_intrinsics_.fy << endl
        << "-- cx: " << left_cam_intrinsics_.cx << endl
        << "-- cy: " << left_cam_intrinsics_.cy << endl;

    cout << "[KittiLoader] LiDAR extrinsics:" << endl
        << T_lc_l_.matrix() << endl;

    size_t lidar_data_num = GetFileNumInDir(lidar_path_);
    size_t left_img_num = GetFileNumInDir(left_img_path_);
    size_t right_img_num = GetFileNumInDir(right_img_path_);

    if (! (lidar_data_num == left_img_num && left_img_num == right_img_num)){
        cerr << "[KittiLoader] Dataset not complete, frame number of LiDAR, "
             << "left camera and right camera not equal" << endl; 
        if_success = false;
    }
    
    size_ = lidar_data_num;

    // Still consider as success if only groundtruth not available
    if(!exists(gt_pose_path_)){
        cerr << "[KittiLoader] Gorundtruth pose for each frame is not available" << endl;
        gt_available_ = false;
    }else{
        LoadGroundtruthPose(gt_pose_path_.string());
        gt_available_ = true;
    }

    if_success = true;
}

Frame KittiLoader::operator[](size_t i) const{
    using boost::format;
    // format fmt_lidar("%s/%06d.bin");
    format fmt_lidar("%s/%06d.pcd");
    format fmt_img("%s/%06d.png");

    string ptcloud = (fmt_lidar % lidar_path_.string() % i).str();
    string left_img = (fmt_img % left_img_path_.string() % i).str();
    string right_img = (fmt_img % right_img_path_.string() % i).str();

    Frame f;
    f.ptcloud = LoadPtCloud(ptcloud);
    f.left_img = cv::imread(left_img, cv::IMREAD_GRAYSCALE);
    f.right_img = cv::imread(right_img, cv::IMREAD_GRAYSCALE);

    if(gt_available_){
        f.gt_pose = gt_poses_[i];
    }

    return f;
}

// Reference: https://stackoverflow.com/questions/41304891/how-to-count-the-number-of-files-in-a-directory-using-standard
size_t KittiLoader::GetFileNumInDir(const boost::filesystem::path &p){
    using boost::filesystem::directory_iterator;
    return std::distance(directory_iterator(p), directory_iterator{});
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KittiLoader::LoadPtCloud(const string &path){
    using pcl::PointCloud;
    using pcl::PointXYZI;

    ifstream lidar_data_file(path, ifstream::in | ifstream::binary);

    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data.front()), 
                         num_elements * sizeof(float));

    PointCloud<PointXYZI>::Ptr lidar_frame_ptr(new PointCloud<PointXYZI>);
    for(size_t i = 0; i < lidar_data.size(); i += 4){
        PointXYZI p;
        p.x = lidar_data[i];
        p.y = lidar_data[i + 1];
        p.z = lidar_data[i + 2];
        p.intensity = lidar_data[i + 3];
        lidar_frame_ptr->push_back(p);
    }

    lidar_frame_ptr->width = lidar_frame_ptr->size();
    lidar_frame_ptr->height = 1;
    lidar_frame_ptr->is_dense = true;

    return lidar_frame_ptr;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KittiLoader::ConcatePtCloud(size_t start_idx, unsigned int num_of_cloud){
    using pcl::PointCloud;
    using pcl::PointXYZI;

    if(!gt_available_){
        cerr << "[KittiLoader] Groundtruth not available, cannot perform concatation" << endl;
        return nullptr;
    }

    if(start_idx >= size_){
        cerr << "[KittiLoader] Start index out of bound" << endl;
        return nullptr;
    }

    size_t end_idx = start_idx + num_of_cloud;
    if(end_idx >= size_){
        end_idx = size_;
    }

    using boost::format;
    format fmt_lidar("%s/%06d.bin");

    PointCloud<PointXYZI>::Ptr result(new PointCloud<PointXYZI>);
    for(unsigned int i = start_idx; i < end_idx; ++i){
        string ptcloud_file = (fmt_lidar % lidar_path_.string() % i).str();
        PointCloud<PointXYZI>::Ptr ptcloud = LoadPtCloud(ptcloud_file);

        // Transform to world frame
        pcl::transformPointCloud(*ptcloud, *ptcloud, (gt_poses_[i] * T_lc_l_).matrix());
        *result += *ptcloud;
    }

    // Transform back to frame at start_idx
    pcl::transformPointCloud(*result, *result, (gt_poses_[start_idx] * T_lc_l_).inverse().matrix());

    return result;
}

void KittiLoader::LoadCaliData(const std::string& path) {
    ifstream fin(path);

    for (int i = 0; i < 4; ++i) {
        char cam_name[3];
        for (int j = 0; j < 3; ++j) {
            fin >> cam_name[j];
        }

        double projection_data[12];
        for (int j = 0; j < 12; ++j) {
            fin >> projection_data[j];
        }

        // Here we only load the left camera intrinsics
        // Intrinsics matrix:
        //   projection_data[0], projection_data[1], projection_data[2],
        //   projection_data[4], projection_data[5], projection_data[6],
        //   projection_data[8], projection_data[9], projection_data[10];
        if (i == 0) {
            left_cam_intrinsics_.fx = projection_data[0];
            left_cam_intrinsics_.fy = projection_data[5];
            left_cam_intrinsics_.cx = projection_data[2];
            left_cam_intrinsics_.cy = projection_data[6];
        }
    }

    // Load extrinsics (T_lc_l)
    char trans_prefix[3];
    for (int k = 0; k < 3; ++k) {
        fin >> trans_prefix[k];
    }

    double lidar_trans_data[12];
    for (int k = 0; k < 12; ++k) {
        fin >> lidar_trans_data[k];
    }

    Eigen::Matrix3f rotation;
    rotation << lidar_trans_data[0], lidar_trans_data[1], lidar_trans_data[2],
                lidar_trans_data[4], lidar_trans_data[5], lidar_trans_data[6],
                lidar_trans_data[8], lidar_trans_data[9], lidar_trans_data[10];

    Eigen::Vector3f translation;
    translation << lidar_trans_data[3], lidar_trans_data[7], lidar_trans_data[11];

    T_lc_l_ = Eigen::Isometry3f(rotation);
    T_lc_l_.pretranslate(translation);
}

void KittiLoader::LoadGroundtruthPose(const std::string& path){
    ifstream fin(path);
    for(int i = 0; i < size_; ++i){
        float data[12];
        fin >> data[0] >> data[1] >> data[2] >> data[3] >>
               data[4] >> data[5] >> data[6] >> data[7] >>
               data[8] >> data[9] >> data[10] >> data[11];
        
        Eigen::Matrix4f m;
        m << data[0], data[1], data[2], data[3],
             data[4], data[5], data[6], data[7],
             data[8], data[9], data[10], data[11],
             0.0f, 0.0f, 0.0f, 1.0f;
        gt_poses_.push_back(Eigen::Isometry3f(m));
    }
}