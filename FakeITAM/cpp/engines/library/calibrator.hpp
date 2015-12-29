//
//  calibrator.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/5.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_CALIBRATOR_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_CALIBRATOR_HPP_

#include "utilities/matrix.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

class RGBDCalibrator {
 public:
  RGBDCalibrator(const char* calib_filename);
  RGBDCalibrator(const RGBDCalibrator& other);
  ~RGBDCalibrator();

  int rgb_width() const { return rgb_width_; }
  int rgb_height() const { return rgb_height_; }
  int depth_width() const { return depth_width_; }
  int depth_height() const { return depth_height_; }

  float disparity_x() const { return disparity_x_; }
  float disparity_y() const { return disparity_y_; }

  const utility::Matrix3f* rgb_intrinsics() const {
    return rgb_intrinsics_;
  }

  const utility::Matrix3f* depth_intrinsics() const {
    return depth_intrinsics_;
  }

  const utility::Matrix4f* rgb_to_depth_extrinsics() const {
    return rgb_to_depth_extrinsics_;
  }

  utility::Vector4f GetRGBIntrinsicVector() const {
    return utility::Vector4f((*rgb_intrinsics_)(0, 0),
                             (*rgb_intrinsics_)(1, 1),
                             (*rgb_intrinsics_)(0, 2),
                             (*rgb_intrinsics_)(1, 2));
  }

  utility::Vector4f GetDepthIntrinsicVector() const {
    return utility::Vector4f((*depth_intrinsics_)(0, 0),
                             (*depth_intrinsics_)(1, 1),
                             (*depth_intrinsics_)(0, 2),
                             (*depth_intrinsics_)(1, 2));
  }

 private:
  /* RGB & Depth intrinsics */
  int rgb_width_, rgb_height_;
  int depth_width_, depth_height_;

  /* Disparity Calibration Parameters */
  float disparity_x_, disparity_y_;

  /* Intrinsic matrices */
  utility::Matrix3f* rgb_intrinsics_;
  utility::Matrix3f* depth_intrinsics_;
  utility::Matrix4f* rgb_to_depth_extrinsics_;
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_CALIBRATOR_HPP_ */
