#ifndef __KITTI_LOADER__
#define __KITTI_LOADER__

#include <string>
#include <boost/filesystem/path.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kitti{

    struct Frame {
        cv::Mat left_img;
        cv::Mat right_img;
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud;
        Eigen::Isometry3f gt_pose; // Transform from current left camera frame to world frame (first frame)
    };

    struct Intrinsics {
        float fx;
        float fy;
        float cx;
        float cy;
    };

    class KittiLoader {
    public:
        using Ptr = std::shared_ptr<KittiLoader>;

        KittiLoader(const std::string& data_root, bool& if_success);

        inline size_t Size() const { return size_; }
        inline Intrinsics GetLeftCamIntrinsics() const { return left_cam_intrinsics_; }
        inline Eigen::Isometry3f GetExtrinsics() const { return T_lc_l_; }

        Frame operator[](size_t i) const;

        static pcl::PointCloud<pcl::PointXYZI>::Ptr LoadPtCloud(const std::string& path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ConcatePtCloud(size_t start_idx, unsigned int num_of_cloud);

    private:
        size_t GetFileNumInDir(const boost::filesystem::path& p);
        void LoadCaliData(const std::string& path);
        void LoadGroundtruthPose(const std::string& path);

        boost::filesystem::path root_;
        boost::filesystem::path lidar_path_;
        boost::filesystem::path left_img_path_;
        boost::filesystem::path right_img_path_;
        boost::filesystem::path cali_file_path_;
        boost::filesystem::path gt_pose_path_;

        bool gt_available_ = false;

        // Transform from current left camera frame to world frame (first frame)
        std::vector<Eigen::Isometry3f> gt_poses_;

        Intrinsics left_cam_intrinsics_;

        // Transform from lidar to left camera
        Eigen::Isometry3f T_lc_l_;

        size_t size_ = 0;
    };


};

#endif