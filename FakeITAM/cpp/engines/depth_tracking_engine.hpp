//
//  DepthTrackingEngine.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/21.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_DEPTH_TRACKING_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_DEPTH_TRACKING_ENGINE_HPP_

#include "engines/tracking_engine.hpp"
#include "utilities/matrix.hpp"
#include "utilities/mem_block.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

struct CameraPose;
class PointCloud;
class ViewPyramid;

class DepthTrackingEngine : public TrackingEngine {
 public:
  DepthTrackingEngine() : view_pyramid_(nullptr) {}
  virtual ~DepthTrackingEngine();
  virtual void TrackCamera(const View& view_in,
                           const PointCloud& global_pcl_in,
                           const CameraPose& pose_in,
                                 CameraPose* pose_out);

  const ViewPyramid* view_pyramid() const { return view_pyramid_; }

 private:
  void SigmaAtAAndSigmaAtB(const View& view_in,
                           const utility::Matrix4f& Tg_last_estimate_in,
                           const utility::Matrix4f& Ti_g_last_view_in,
                           const PointCloud& global_pcl_in,
                                 float dist_threshold_in,
                                 utility::Matrix<float, 6, 6>* sigma_At_A_out,
                                 utility::Matrix<float, 6, 1>* sigma_At_b_out,
                                 int* valid_pixel_num_out,
                                 float* error_function_out) const;

  void ComputeAtAndB(const utility::Matrix4f& Tg_last_estimate_in,
                     const utility::Matrix4f& Ti_g_last_view_in,
                     const utility::Vector2i& view_size_in,
                     const utility::Vector4f& view_intrinsics_in,
                     const utility::MemBlock<float>& view_depth_in,
                     const PointCloud& global_pcl_in,
                           float dist_threshold_in, int x, int y,
                           utility::Matrix<float, 6, 1>* At_out,
                           utility::Matrix<float, 1, 1>* b_out,
                           bool* valid_out, float* error_function_out) const;

  void ComputeX(const utility::Matrix<float, 6, 6>& At_A_in,
                const utility::Matrix<float, 6, 1>& At_b_in,
                      utility::Matrix<float, 6, 1>* x_out) const;
  void UpdateEstimateT(const utility::Matrix<float, 6, 1>& x_in,
                       const utility::Matrix<float, 4, 4>& Tg_last_estimate_in,
                             utility::Matrix<float, 4, 4>* Tg_estimate_out) const;
  bool IsConverged(const utility::Matrix<float, 6, 1>& x_in) const;

  ViewPyramid* view_pyramid_;

  DepthTrackingEngine(const DepthTrackingEngine&);
  DepthTrackingEngine& operator=(const DepthTrackingEngine&);
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_DEPTH_TRACKING_ENGINE_HPP_ */
