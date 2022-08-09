#include <iostream>

#include <glog/logging.h>

#include <pcl/io/pcd_io.h>

#include "kitti_loader/KittiLoader.hpp"

#include "Calibrator.hpp"
#include "Core.hpp"
#include "Rand.hpp"
#include "Misc.hpp"

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

    kitti::Intrinsics intri = kitti_dataset.GetLeftCamIntrinsics();

    alcc::Calibrator calibrator;
    calibrator.SetCameraIntrinsic(intri.fx * 0.3f, intri.fy * 0.3f, intri.cx * 0.3f, intri.cy * 0.3f);
    calibrator.SetMaxFrameNum(5);

    Eigen::Isometry3f origin_extri = kitti_dataset.GetExtrinsics();
    

    m2d::DataManager::SetRoot("E:/exp_data/comp_exp/kitti_00_0_1000");
    alcc::FrameNumVec6DataBag::Ptr bag_track(new alcc::FrameNumVec6DataBag("thrun_1"));
    m2d::DataManager::Add(bag_track);


    for (int i = 500; i < 507; ++i) {
        kitti::Frame f = kitti_dataset[i];
        cv::resize(f.left_img, f.left_img, cv::Size(), 0.3, 0.3);
        calibrator.AddDataFrame(f.ptcloud, f.left_img);
    }

    std::vector<Eigen::Isometry3f> errs = alcc::Rand::MultiUniformIsometry3f(2.0f, 0.2f, 10);
    for (int i = 0; i < errs.size(); ++i) {
        Eigen::Isometry3f err_extri = errs[i] * origin_extri;
        Eigen::Isometry3f track_extri;
        calibrator.CalibrationTrack(err_extri, track_extri, 10);

        alcc::Vector6f err_track_vec = alcc::GetIsometryErr(origin_extri, track_extri);
        bag_track->AddData(i, err_track_vec);
    }

    // Dump rotation in radians
    m2d::DataManager::DumpAll();

    // Eigen::Isometry3f result;
    // calibrator.CalibrationTrack(kitti_dataset.GetExtrinsics(), result, 10);
    // LOG(INFO) << result.matrix();

#if 0

    kitti::Frame f = kitti_dataset[0];

    cv::Mat edge_img;
    alcc::GenEdgeImage(f.left_img, edge_img);

    cv::resize(edge_img, edge_img, cv::Size(), 0.3, 0.3);

    cv::imshow("edge_img", edge_img);
    cv::waitKey(0);

    cv::Mat inverse_img;
    alcc::InverseDistTransform(edge_img, inverse_img);

    inverse_img.convertTo(inverse_img, CV_8UC1);

    cv::imshow("img", inverse_img);
    cv::waitKey(0);

    //-------------

    alcc::PtCloudXYZI_Type::Ptr discon_cloud = alcc::GenDiscontinuityCloud(*f.ptcloud);

    pcl::io::savePCDFileBinary("E://discon_cloud.pcd", *discon_cloud);

    alcc::CloudDiscontinuityFilter(*discon_cloud, 0.5f);

    pcl::io::savePCDFileBinary("E://discon_cloud_filtered.pcd", *discon_cloud);

    // ------------

    

#endif

    

    return EXIT_SUCCESS;
}