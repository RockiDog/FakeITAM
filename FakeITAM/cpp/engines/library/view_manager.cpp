//
//  view_manager.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/14.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/library/view_manager.hpp"

#include "engines/library/calibrator.hpp"
#include "utilities/matrix.hpp"
#include "utilities/vector.hpp"

using namespace fakeitam::utility;
using namespace fakeitam::engine;

const float ViewManager::kSigmaL = 1.2232;

View::View(const ImageRGB8u* rgb_image, const ImageMono16u* disparity_map,
           const Vector2i& size, const Vector4f& intrinsics)
    : rgb_image(rgb_image), disparity_map(disparity_map), size(size), intrinsics(intrinsics) {
  if (disparity_map != nullptr) {
    depth_map = new ImageMono32f(disparity_map->element_n(), MEM_CPU);
    depth_weights = new ImageMono32f(disparity_map->element_n(), MEM_CPU);
    normal_map = new NormalXYZW32f(disparity_map->element_n(), MEM_CPU);
  } else {
    depth_map = nullptr;
    depth_weights = nullptr;
    normal_map = nullptr;
  }
}

View::View(const View& other)
    : rgb_image(other.rgb_image), disparity_map(other.disparity_map),
      size(other.size), intrinsics(other.intrinsics) {
  if (other.depth_map != nullptr)
    depth_map = new ImageMono32f(*other.depth_map);
  else
    depth_map = nullptr;
  if (other.depth_weights != nullptr)
    depth_weights = new ImageMono32f(*other.depth_weights);
  else
    depth_weights = nullptr;
  if (other.normal_map != nullptr)
    normal_map = new NormalXYZW32f(*other.normal_map);
  else
    normal_map = nullptr;
}

View::~View() {
  if (depth_map != nullptr)
    delete depth_map;
  if (depth_weights != nullptr)
    delete depth_weights;
  if (normal_map != nullptr)
    delete normal_map;
  depth_map = nullptr;
  depth_weights = nullptr;
  normal_map = nullptr;
}

void ViewManager::UpdateView(View* view) {
  const ImageMono16u* disparity_map = view->disparity_map;
  ImageMono32f* raw_depth_map = view->depth_map;

  ImageMono32f* depth_map = view->depth_map;
  ImageMono32f* depth_weights = view->depth_weights;
  NormalXYZW32f* normal_map = view->normal_map;

  ConvertDisparityToDepth(*disparity_map, raw_depth_map);
  BilateralDepthFilter(*raw_depth_map, depth_map);
  CalculateNormalsAndWeights(*depth_map, normal_map, depth_weights);
}

/* TODO Metal */
void ViewManager::ConvertDisparityToDepth(const ImageMono16u& disparity_in, ImageMono32f* depth_out) {
  const Matrix3f& intrinsics = *(calibrator_.depth_intrinsics());
  float fx_depth = intrinsics(0, 0);
  float disparity_x = calibrator_.disparity_x();
  float disparity_y = calibrator_.disparity_y();

  for (int i = 0; i < disparity_in.element_n(); ++i) {
    unsigned short disparity = disparity_in[i];
    float depth = 0;
    if (disparity_x != disparity)
      depth = (8 * disparity_y * fx_depth) / (disparity_x - disparity);
    (*depth_out)[i] = depth > 0 ? depth : -1;
  }
}

void ViewManager::BilateralDepthFilter(const ImageMono32f& depth_in, ImageMono32f* depth_out) {
  ImageMono32f temp_depth = depth_in;

  /* 5 steps of bilateral filtering */
  BilateralFilter(temp_depth, depth_out);
  BilateralFilter(*depth_out, &temp_depth);
  BilateralFilter(temp_depth, depth_out);
  BilateralFilter(*depth_out, &temp_depth);
  BilateralFilter(temp_depth, depth_out);
}

/* TODO Metal */
void ViewManager::BilateralFilter(const ImageMono32f& data_in, ImageMono32f* data_out) {
  data_out->ResetData();
  int width = calibrator_.depth_width();
  int height = calibrator_.depth_height();

  for (int row = 0; row < height; ++row)
    for (int col = 0; col < width; ++col)
      (*data_out)[row * width + col] = CalculateConvolution(data_in, row, col);
}

/* TODO Metal */
void ViewManager::CalculateNormalsAndWeights(const ImageMono32f& depth_in,
                                                   NormalXYZW32f* normals_out,
                                                   ImageMono32f* weights_out) {
  int width = calibrator_.depth_width();
  int height = calibrator_.depth_height();

  for (int i = kKernalSize / 2; i < height - kKernalSize / 2; ++i) {
    for (int j = kKernalSize / 2; j < width - kKernalSize / 2; ++j) {
      CalculateNormal(depth_in, normals_out, i, j);
      CalculateWeight(*normals_out, weights_out, i, j);
    }
  }
}
