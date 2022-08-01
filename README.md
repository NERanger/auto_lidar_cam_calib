# auto_lidar_cam_calib

A reimplementation for Levinson, J., &amp; Thrun, S. (2016). Automatic Online Calibration of Cameras and Lasers. https://doi.org/10.15607/rss.2013.ix.029

## Requirement

* [PCL](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)
* [OpenCV](https://opencv.org/)
* [glog](https://github.com/google/glog)

## Usage

```cpp
// If log output is desired
FLAGS_logtostderr = 1;
google::InitGoogleLogging(argv[0]);

alcc::Calibrator calibrator;
calibrator.SetCameraIntrinsic(<intrinsic parameters>);
calibrator.SetMaxFrameNum(<max frame number>);

calibrator.AddDataFrame(<ptcloud>, <img>);

// Miscalibration detection
// score in [0.0 - 1.0], higher is better
float score = calibrator.MiscalibrationDetection(<current extrinsic>);

// Track calibration
Eigen::Isometry3f result;
calibrator.CalibrationTrack(<current extrinsic>, <result>, <iteration number>);
```

A sample with kitti dataset is given in samples directory.