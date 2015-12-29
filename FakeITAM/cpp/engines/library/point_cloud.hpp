//
//  point_cloud.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/24.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_HPP_

#include "global_config.hpp"
#include "engines/library/camera_pose.hpp"
#include "engines/library/image_utils.hpp"
#include "utilities/mem_block.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

class PointCloud {
 public:
  PointCloud(const utility::Vector2i& size, utility::MemDevice device)
      : age_(config::gRenderMaxPointCloudAge), size_(size) {
    locations_ = new utility::MemBlock<utility::Vector4f>(size_.x * size_.y, device);
    normals_ = new utility::MemBlock<utility::Vector4f>(size_.x * size_.y, device);
  }

  ~PointCloud() {
    delete locations_;
    delete normals_;
    locations_ = nullptr;
    normals_ = nullptr;
  }

  int age() const { return age_; }
  const CameraPose& camera_pose() const { return camera_pose_; }
  void set_camera_pose(const CameraPose& pose) { camera_pose_ = pose; }
  const utility::MemBlock<utility::Vector4f>* locations() const { return locations_; }
  const utility::MemBlock<utility::Vector4f>* normals() const { return normals_; }
  utility::MemBlock<utility::Vector4f>* locations() { return locations_; }
  utility::MemBlock<utility::Vector4f>* normals() { return normals_; }

  utility::Matrix3f R() const { return camera_pose_.R(); }
  utility::Vector3f t() const { return camera_pose_.t(); }

  void IncreaseAge() { ++age_; }
  void ResetAge() { age_ = 0; }

 private:
  int age_;
  const utility::Vector2i size_;
  CameraPose camera_pose_;
  utility::MemBlock<utility::Vector4f>* locations_;
  utility::MemBlock<utility::Vector4f>* normals_;

  PointCloud(const PointCloud&);
  PointCloud& operator=(const PointCloud&);
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_HPP_ */
