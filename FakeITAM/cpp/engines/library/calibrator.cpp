//
//  calibrator.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/5.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/library/calibrator.hpp"

#include <fstream>

using namespace std;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

RGBDCalibrator::RGBDCalibrator(const char* calib_filename) 
    : rgb_intrinsics_(nullptr),
      depth_intrinsics_(nullptr),
      rgb_to_depth_extrinsics_(nullptr) {
  ifstream ifs(calib_filename);

  float rgb_fx_, rgb_fy_, rgb_cx_, rgb_cy_;
  float depth_fx_, depth_fy_, depth_cx_, depth_cy_;
  ifs >> rgb_width_ >> rgb_height_;
  ifs >> rgb_fx_ >> rgb_fy_ >> rgb_cx_ >> rgb_cy_;
  ifs >> depth_width_ >> depth_height_;
  ifs >> depth_fx_ >> depth_fy_ >> depth_cx_ >> depth_cy_;

  rgb_intrinsics_ = new Matrix3f {
    rgb_fx_, 0, rgb_cx_,
    0, rgb_fy_, rgb_cy_,
    0, 0, 1
  };
  depth_intrinsics_ = new Matrix3f {
    depth_fx_, 0, depth_cx_,
    0, depth_fy_, depth_cy_,
    0, 0, 1
  };
  rgb_to_depth_extrinsics_ = new Matrix4f;
  Matrix4f& ex = *rgb_to_depth_extrinsics_;
  ifs >> ex(0, 0) >> ex(0, 1) >> ex(0, 2) >> ex(0, 3)
      >> ex(1, 0) >> ex(1, 1) >> ex(1, 2) >> ex(1, 3)
      >> ex(2, 0) >> ex(2, 1) >> ex(2, 2) >> ex(2, 3);
  ex(3, 0) = ex(3, 1) = ex(3, 2) = 0;
  ex(3, 3) = 1;

  ifs >> disparity_x_ >> disparity_y_;

  ifs.close();
}

RGBDCalibrator::RGBDCalibrator(const RGBDCalibrator& other)
    : rgb_width_(other.rgb_width_), rgb_height_(other.rgb_height_),
      depth_width_(other.depth_width_), depth_height_(other.depth_height_),
      disparity_x_(other.disparity_x_), disparity_y_(other.disparity_y_) {
  rgb_intrinsics_ = new Matrix3f(*other.rgb_intrinsics_);
  depth_intrinsics_ = new Matrix3f(*other.depth_intrinsics_);
  rgb_to_depth_extrinsics_ = new Matrix4f(*other.rgb_to_depth_extrinsics_);
}

RGBDCalibrator::~RGBDCalibrator() {
  if (rgb_intrinsics_ != nullptr)
    delete rgb_intrinsics_;
  rgb_intrinsics_ = nullptr;
  if (depth_intrinsics_ != nullptr)
    delete depth_intrinsics_;
  depth_intrinsics_ = nullptr;
  if (rgb_to_depth_extrinsics_ != nullptr)
    delete rgb_to_depth_extrinsics_;
  rgb_to_depth_extrinsics_ = nullptr;
}

