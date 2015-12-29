//
//  view_pyramid.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/27.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/library/view_pyramid.hpp"

#include "global_config.hpp"
#include "utilities/vector.hpp"

using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

ViewPyramid::ViewPyramid(int level_n, float top_icp_threshold, const View& bottom)
    : level_n_(level_n) {
  pyramid_ = new View*[level_n];
  for (int i = 0; i < level_n; ++i)
    pyramid_[i] = nullptr;
  bottom_ = new View(bottom);
  FileterSubsample();

  icp_thresholds = new float[level_n];
  float step = top_icp_threshold / level_n;
  icp_thresholds[level_n - 1] = top_icp_threshold;
  for (int i = level_n - 2; i >= 0; --i)
    icp_thresholds[i] = icp_thresholds[i + 1] - step;
}

ViewPyramid::~ViewPyramid() {
  for (int i = 0; i < level_n_; ++i) {
    if (pyramid_[i] != nullptr)
      delete pyramid_[i];
    pyramid_[i] = nullptr;
  }
  delete [] pyramid_;
  delete [] icp_thresholds;
  pyramid_ = nullptr;
  icp_thresholds = nullptr;
}

void ViewPyramid::FileterSubsample() {
  if (bottom_ == nullptr)
    return;
  pyramid_[0] = bottom_;
  for (int i = 1; i < level_n_; ++i) {
    pyramid_[i] = new View(nullptr, nullptr, pyramid_[i - 1]->size / 2, pyramid_[i - 1]->intrinsics / 2);
    subsample_with_holes(pyramid_[i - 1]->depth_map, &pyramid_[i]->depth_map, pyramid_[i - 1]->size);
  }
}

void ViewPyramid::subsample_with_holes(const ImageMono32f* depth_in,
                                             ImageMono32f** depth_out,
                                             utility::Vector2i origin_size) {
  Vector2i new_size = origin_size / 2;
  for (int i = 0; i < new_size.y; ++i) {
    for (int j = 0; j < new_size.x; ++j) {
      float depth_src, depth = 0;
      int valid_pixel_num = 0;
      int src_row = i * 2, src_col = j * 2;
      
      depth_src = (*depth_in)[(src_row + 0) * origin_size.x + src_col + 0];
      if (depth_src > 0) {
        depth += depth_src;
        ++valid_pixel_num;
      }
      
      depth_src = (*depth_in)[(src_row + 1) * origin_size.x + src_col + 0];
      if (depth_src > 0) {
        depth += depth_src;
        ++valid_pixel_num;
      }
      
      depth_src = (*depth_in)[(src_row + 1) * origin_size.x + src_col + 1];
      if (depth_src > 0) {
        depth += depth_src;
        ++valid_pixel_num;
      }
      
      depth_src = (*depth_in)[(src_row + 0) * origin_size.x + src_col + 1];
      if (depth_src > 0) {
        depth += depth_src;
        ++valid_pixel_num;
      }
      
      (*depth_out) = new ImageMono32f(new_size.x * new_size.y, MEM_CPU);
      (**depth_out)[i * new_size.x + j] = depth / valid_pixel_num;
    }
  }
}
