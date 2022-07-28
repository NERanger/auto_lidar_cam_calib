#include <iostream>

#include <glog/logging.h>

#include <pcl/io/pcd_io.h>

#include "kitti_loader/KittiLoader.hpp"

#include "Core.hpp"

int main(int argc, char const* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: ./discontinuity_cloud_gen <path-to-kitti-dataset>" << std::endl;
        return EXIT_FAILURE;
    }

    FLAGS_logtostderr = 1;
    google::InitGoogleLogging(argv[0]);

    bool kitti_success = false;
    std::string kitti_path(argv[1]);
    kitti::KittiLoader kitti_dataset(kitti_path, kitti_success);
    CHECK(kitti_success);

    kitti::Frame f = kitti_dataset[300];

#if 0

    alcc::PtCloudXYZI_Type::Ptr discon_cloud = alcc::GenDiscontinuityCloud(*f.ptcloud);

    pcl::io::savePCDFileBinary("E://discon_cloud.pcd", *discon_cloud);

    alcc::CloudDiscontinuityFilter(*discon_cloud, 0.3f);

    pcl::io::savePCDFileBinary("E://discon_cloud_filtered.pcd", *discon_cloud);

#endif

    cv::Mat edge_img;
    alcc::GenEdgeImage(f.left_img, edge_img);

    cv::resize(edge_img, edge_img, cv::Size(), 0.3, 0.3);

    cv::imshow("edge_img", edge_img);
    cv::waitKey(0);

    cv::Mat inverse_img;
    alcc::InverseDistTransform(edge_img, inverse_img);

    cv::imshow("img", inverse_img);
    cv::waitKey(0);

    return EXIT_SUCCESS;
}