//
//  view_manager.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/14.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_VIEW_MANAGER_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_VIEW_MANAGER_HPP_

#include <cmath>

#include "engines/library/calibrator.hpp"
#include "engines/library/image_utils.hpp"
#include "utilities/mem_block.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

struct View {
  View(const ImageRGB8u* rgb_image, const ImageMono16u* disparity_map,
       const utility::Vector2i& size, const utility::Vector4f& intrinsics);
  View(const View& other);
  ~View();

  /* Input rgb image & depth map */
  const ImageRGB8u* rgb_image;
  const ImageMono16u* disparity_map;

  /* Filtered depth map and corresponding weights & normals */
  ImageMono32f* depth_map;
  ImageMono32f* depth_weights;
  NormalXYZW32f* normal_map;

  utility::Vector2i size;
  utility::Vector4f intrinsics;

 private:
  View& operator=(const View&);
};

class ViewManager {
 public:
  ViewManager(const RGBDCalibrator& calibrator, int kernal_size)
      : calibrator_(calibrator), kKernalSize(kernal_size) {}

  void UpdateView(View* view);

 private:
  void ConvertDisparityToDepth(const ImageMono16u& disparity_in, ImageMono32f* depth_out);
  void BilateralDepthFilter(const ImageMono32f& depth_in, ImageMono32f* depth_out);
  void CalculateNormalsAndWeights(const ImageMono32f& depth_in, NormalXYZW32f* normals_out,
                                        ImageMono32f* weights_out);

  void BilateralFilter(const ImageMono32f& data_in, ImageMono32f* data_out);
  float CalculateConvolution(const ImageMono32f& depth_in, int row, int col);
  void CalculateNormal(const ImageMono32f& data_in, NormalXYZW32f* normals_out, int row, int col);
  void CalculateWeight(const NormalXYZW32f& normals_in, ImageMono32f* weights_out, int row, int col);

  RGBDCalibrator calibrator_;
  const int kKernalSize;
  static const float kSigmaL;

  ViewManager(const ViewManager&);
  ViewManager& operator=(const ViewManager&);
};

inline float ViewManager::CalculateConvolution(const ImageMono32f& depth_in, int row, int col) {
  int width = calibrator_.depth_width();
  int height = calibrator_.depth_height();
  float raw_z = depth_in[row * width + col];
  if (raw_z <= 0)
    return -1;

  if (row >= kKernalSize / 2 && row < height - kKernalSize / 2 &&
      col >= kKernalSize / 2 && col < width - kKernalSize / 2) {
    float sigma_z = 1 / (0.0012 + 0.0019 * pow((raw_z - 0.4), 2) + 0.0001 / sqrt(raw_z) * 0.25);
    float z_sum = 0, w_sum = 0;
    for (int i = -kKernalSize / 2; i <= kKernalSize / 2; ++i) {
      for (int j = -kKernalSize / 2; j <= kKernalSize / 2; ++j) {
        float z = depth_in[(row + i) * width + col + j];
        if (z <= 0)
          continue;
        float delta_z = z - raw_z;
        float w = exp(-0.5 * ((abs(i) + abs(j)) * kSigmaL * kSigmaL + delta_z * delta_z * sigma_z * sigma_z));
        z_sum += z * w;
        w_sum += w;
      }
    }
    return z_sum / w_sum;
  } else {
    return depth_in[row * width + col];
  }
}

inline void ViewManager::CalculateNormal(const ImageMono32f& depth_in,
                                               NormalXYZW32f* normals_out,
                                               int row, int col) {
  int width = calibrator_.depth_width();
  int id = row * width + col;
  float depth = depth_in[id];
  if (depth <= 0) {
    (*normals_out)[id].w = -1;
    return;
  }

  const float fx = (*calibrator_.depth_intrinsics())(0, 0);
  const float fy = (*calibrator_.depth_intrinsics())(1, 1);
  const float cx = (*calibrator_.depth_intrinsics())(0, 2);
  const float cy = (*calibrator_.depth_intrinsics())(1, 2);

  utility::Vector3f up, down, left, right;
  up.z = depth_in[(row - 1) * width + col];
  down.z = depth_in[(row + 1) * width + col];
  left.z = depth_in[row * width + col - 1];
  right.z = depth_in[row * width + col + 1];
  if (up.z <= 0 || down.z <= 0 || left.z <= 0 || right.z <= 0) {
    (*normals_out)[id].w = -1;
    return;
  }

  up = utility::Vector3f((col - cx) / fx, (row - 1 - cy) / fy, 1) * up.z;
  down = utility::Vector3f((col - cx) / fx, (row + 1 - cy) / fy, 1) * down.z;
  left = utility::Vector3f((col - 1 - cx) / fx, (row - cy) / fy, 1) * left.z;
  right = utility::Vector3f((col + 1 - cx) / fx, (row - cy) / fy, 1) * right.z;

  utility::Vector3f up_down = down - up;
  utility::Vector3f left_right = right - left;

  utility::Vector4f normal;
  normal.x = up_down.y * left_right.z - up_down.z * left_right.y;
  normal.y = up_down.z * left_right.x - up_down.x * left_right.z;
  normal.z = up_down.x * left_right.y - up_down.y * left_right.x;
  if (normal.x <= 0 && normal.y <= 0 && normal.z <=0) {
    normal.w = -1;
    return;
  }

  float mod = sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
  normal = normal / mod;
  normal.w = 1;
  (*normals_out)[id] = normal;
}

inline void ViewManager::CalculateWeight(const NormalXYZW32f& normals_in,
                                               ImageMono32f* weights_out,
                                               int row, int col) {
  int width = calibrator_.depth_width();
  int id = row * width + col;
  float z = normals_in[id].z;
  if (z <= 0) {
    (*weights_out)[id] = -1;
    return;
  }

  float theta = acos(z);
  float theta_delta = theta / (M_PI / 2 - theta);
  float w = 0.0012 + 0.0019 * pow(z - 0.4, 2) + 0.0001 / sqrt(z) * pow(theta_delta, 2);
  (*weights_out)[id] = w;
}

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_VIEW_MANAGER_HPP_ */
