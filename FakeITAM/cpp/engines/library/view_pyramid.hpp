//
//  view_pyramid.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/27.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_VIEW_PYRAMID_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_VIEW_PYRAMID_HPP_

#include "engines/library/view_manager.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

class ViewPyramid {
 public:
  ViewPyramid(int level_n, float top_icp_threshold, const View& bottom);
  ~ViewPyramid();

  int level_n() const { return level_n_; }
  const View* bottom() const { return bottom_; }

  void FileterSubsample();
  const View* LevelAt(int level) const { return pyramid_[level]; }
  View* LevelAt(int level) { return pyramid_[level]; }
  float IcpThresholdAtLevel(int level) const { return icp_thresholds[level]; }

 private:
  void subsample_with_holes(const ImageMono32f* depth_in, 
                                  ImageMono32f** depth_out,
                                  utility::Vector2i origin_size);

  const int level_n_;
  View** pyramid_;
  View* bottom_;
  float* icp_thresholds;
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_VIEW_PYRAMID_HPP_ */
