#include <iostream>

#include <glog/logging.h>

#include <pcl/io/pcd_io.h>

#include <opencv2/imgproc.hpp>

#include <livox_loader/LivoxLoader.hpp>

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

    std::string livox_path(argv[1]);
    lvx::LivoxLoader livox_loader(livox_path);

    lvx::Intrinsic intri = livox_loader.GetIntrinsic();

    alcc::Calibrator calibrator;
    calibrator.SetCameraIntrinsic(intri.fx * 0.3f, intri.fy * 0.3f, intri.cx * 0.3f, intri.cy * 0.3f);
    calibrator.SetMaxFrameNum(5);

    Eigen::Isometry3f origin_extri = livox_loader.GetExtrinsic();

    m2d::DataManager::SetRoot("D:\\Datasets\\exp\\comp_exp\\livox_0_800");
    alcc::FrameNumVec6DataBag::Ptr bag_track(new alcc::FrameNumVec6DataBag("thrun_0"));
    m2d::DataManager::Add(bag_track);


    for (int i = 500; i < 507; ++i) {
        lvx::Frame f = livox_loader[i];
        cv::resize(f.img, f.img, cv::Size(), 0.3, 0.3);
        calibrator.AddDataFrame(f.ptcloud, f.img);
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

    return EXIT_SUCCESS;
}