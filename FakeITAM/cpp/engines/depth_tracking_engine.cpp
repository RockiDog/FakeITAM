//
//  DepthTrackingEngine.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/21.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/depth_tracking_engine.hpp"

#include <cmath>

#include "global_config.hpp"
#include "engines/library/camera_pose.hpp"
#include "engines/library/image_utils.hpp"
#include "engines/library/point_cloud.hpp"
#include "engines/library/view_pyramid.hpp"
#include "utilities/cholesky.hpp"

using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

/* XXX Different with Infinitam */
void DepthTrackingEngine::TrackCamera(const View& view_in,
                                      const PointCloud& global_pcl_in,
                                      const CameraPose& pose_in,
                                            CameraPose* pose_out) const {
  /* Set up initial parameters */
  ViewPyramid view_pyramid(gDepthTrackIcpLevelNum, gDepthTrackTopIcpThreshold, view_in);
  Matrix4f Ti_g_last_view = GetInverse(pose_in.m);
  Matrix4f Tg_last_estimate = pose_in.m;
  Matrix4f Tg_last_good_estimate;

  Matrix<float, 6, 6> sigma_At_A, At_A;
  Matrix<float, 6, 1> sigma_At_b, At_b;
  Matrix<float, 6, 1> x;
  int valid_pixel_num;
  float last_error_function;
  float error_function;
  float lambda;

  /* ICP */
  for (int level = gDepthTrackIcpLevelNum - 1; level >= 0; --level) {
    /* Set up level parameters */
    int icp_times = gDepthTrackIcpZetaMax[level];
    View* view = view_pyramid.LevelAt(level);
    float threshold = view_pyramid.IcpThresholdAtLevel(level);
    last_error_function = gDepthTrackMaxErrorPerPixel;
    lambda = 1;
    
    for (int z = 1; z <= icp_times; ++z) {
      SigmaAtAAndSigmaAtB(*view, Tg_last_estimate, Ti_g_last_view, global_pcl_in, threshold,
                          &sigma_At_A, &sigma_At_b, &valid_pixel_num, &error_function);
      
      if (valid_pixel_num <= 0 || error_function > last_error_function) {
        /* XXX Why? */
        Tg_last_estimate = Tg_last_good_estimate;
        lambda *= 10;
      } else {
        Tg_last_good_estimate = Tg_last_estimate;
        At_A = sigma_At_A / valid_pixel_num;
        At_b = sigma_At_b / valid_pixel_num;
        last_error_function = error_function;
        lambda /= 10;
      }
      for (int i = 0; i < 6; ++i)
        At_A(i, i) *= 1 + lambda;
      
      ComputeX(At_A, At_b, &x);
      UpdateEstimateT(x, Tg_last_estimate, &Tg_last_estimate);
      
      if (IsConverged(x))
        break;
    }
  }
  pose_out->m = Tg_last_estimate;
}

/* TODO Test */
void DepthTrackingEngine::SigmaAtAAndSigmaAtB(const View& view_in,
                                              const Matrix4f& Tg_last_estimate_in,
                                              const Matrix4f& Ti_g_last_view_in,
                                              const PointCloud& global_pcl_in,
                                                    float dist_threshold,
                                                    Matrix<float, 6, 6>* sigma_At_A_out,
                                                    Matrix<float, 6, 1>* sigma_At_b_out,
                                                    int* valid_pixel_num_out,
                                                    float* error_function_out) const {
  const Vector2i& view_size = view_in.size;
  const Vector4f& view_intrinsics = view_in.intrinsics;
  const MemBlock<float>& view_depth_map = *view_in.depth_map;

  /* For each pixel */
  for (int y = 0; y < view_size.y; ++y) {
    for (int x = 0; x < view_size.x; ++x) {
      Matrix<float, 6, 1> At;
      Matrix<float, 1, 1> b;
      bool is_pixel_valid;
      float error_function;
      
      ComputeAtAndB(Tg_last_estimate_in, Ti_g_last_view_in,
                    view_size, view_intrinsics, view_depth_map,
                    global_pcl_in, dist_threshold, x, y,
                    &At, &b, &is_pixel_valid, &error_function);
      
      if (is_pixel_valid) {
        ++(*valid_pixel_num_out);
        (*error_function_out) += error_function;
        (*sigma_At_A_out) = (*sigma_At_A_out) + At * At.GetTranspose();
        (*sigma_At_b_out) = (*sigma_At_b_out) + At * b;
      }
    }
  }

  /* Too few valid pixels */
  if ((*valid_pixel_num_out) < gDepthTrackMinValidPixelNum)
    (*error_function_out) = gDepthTrackMaxErrorPerPixel;
  else
    (*error_function_out) = sqrt(*error_function_out) / (*valid_pixel_num_out);
}

/* TODO Test */
void DepthTrackingEngine::ComputeAtAndB(const Matrix4f& Tg_last_estimate_in,
                                        const Matrix4f& Ti_g_last_view_in,
                                        const Vector2i& view_size_in,
                                        const Vector4f& view_intrinsics_in,
                                        const MemBlock<float>& view_depth_in,
                                        const PointCloud& global_pcl_in,
                                              float dist_threshold_in, int x, int y,
                                              Matrix<float, 6, 1>* At_out,
                                              Matrix<float, 1, 1>* b_out,
                                              bool* valid_out, float* error_function_out) const {
  int pixel_id = y * view_size_in.x + x;
  float depth = view_depth_in[pixel_id];

  /* Abandon invalid depth */
  if (depth <= gMathFloatEpsilon) {
    (*valid_out) = false;
    return;
  }

  const float fx = view_intrinsics_in.x;
  const float fy = view_intrinsics_in.y;
  const float cx = view_intrinsics_in.z;
  const float cy = view_intrinsics_in.w;

  /* 3D local point, converted from 2D pixel */
  Vector3f point_3d_l;
  point_3d_l.x = depth * (x - cx) / fx;
  point_3d_l.y = depth * (y - cy) / fy;
  point_3d_l.z = depth;

  /* 4D global point estimate */
  Matrix<float, 4, 1> point_4d_estimate_g {point_3d_l.x,
                                           point_3d_l.y,
                                           point_3d_l.z,
                                           1};
  point_4d_estimate_g = Tg_last_estimate_in * point_4d_estimate_g;

  /* Abandon unreliable pixel reprojection */
  Matrix<float, 4, 1> temp_point_4d = Ti_g_last_view_in * point_4d_estimate_g;
  Matrix<float, 4, 1> temp_point_3d {temp_point_4d(0, 0), temp_point_4d(1, 0),
                                     temp_point_4d(2, 0), temp_point_4d(3, 0)};
  Vector2f pixel_2d_reproj;
  pixel_2d_reproj.x = temp_point_3d(0, 0) / temp_point_3d(2, 0) * fx + cx;
  pixel_2d_reproj.y = temp_point_3d(1, 0) / temp_point_3d(2, 0) * fy + cy;
  if (pixel_2d_reproj.x <= -gDepthTrackMaxPixelError ||
      pixel_2d_reproj.y <= -gDepthTrackMaxPixelError ||
      pixel_2d_reproj.x >= view_size_in.x + gDepthTrackMaxPixelError ||
      pixel_2d_reproj.y >= view_size_in.y + gDepthTrackMaxPixelError) {
    (*valid_out) = false;
    return;
  }

  /* 3D global point esitmate */
  Vector3f point_3d_estimate_g;
  point_3d_estimate_g.x = point_4d_estimate_g(0, 0);
  point_3d_estimate_g.y = point_4d_estimate_g(1, 0);
  point_3d_estimate_g.z = point_4d_estimate_g(2, 0);

  /* 3D global point & narmal prediction, from world model */
  const MemBlock<Vector4f>* global_vertices = global_pcl_in.locations();
  const MemBlock<Vector4f>* global_normals = global_pcl_in.normals();
  Vector4f point_4d_prediction_g;
  Vector4f normal_4d_prediction_g;
  point_4d_prediction_g = BilinearInterpolationWithHoles(*global_vertices, pixel_2d_reproj, view_size_in);
  normal_4d_prediction_g = BilinearInterpolationWithHoles(*global_normals, pixel_2d_reproj, view_size_in);
  if (normal_4d_prediction_g.w < 0 || point_4d_prediction_g.w < 0) {
    (*valid_out) = false;
    return;
  }
  Vector3f point_3d_prediction_g(point_4d_prediction_g.x, point_4d_prediction_g.y, point_4d_prediction_g.z);
  Vector3f normal_prediction_g(normal_4d_prediction_g.x, normal_4d_prediction_g.y, normal_4d_prediction_g.z);

  Vector3f point_3d_diff = point_3d_prediction_g - point_3d_estimate_g;
  float dist = point_3d_diff.GetNorm2();
  if (dist >= dist_threshold_in + gMathFloatEpsilon) {
    (*valid_out) = false;
    return;
  }

  Matrix<float, 3, 6> G {
    0, -point_3d_estimate_g.z, point_3d_estimate_g.y, 1, 0, 0,
    point_3d_estimate_g.z, 0, -point_3d_estimate_g.x, 0, 1, 0,
    -point_3d_estimate_g.y, point_3d_estimate_g.x, 0, 0, 0, 1,
  };
  Matrix<float, 6, 3> Gt = G.GetTranspose();
  (*At_out) = Gt * (Matrix<float, 3, 1>)(normal_prediction_g);

  Matrix<float, 3, 1> temp_point_3d_diff = point_3d_diff;
  Matrix<float, 3, 1> temp_normal_prediction_g = normal_prediction_g;
  (*b_out) = temp_normal_prediction_g.GetTranspose() * temp_point_3d_diff;
  (*error_function_out) = ((*b_out) * (*b_out))(0, 0);
}

void DepthTrackingEngine::ComputeX(const Matrix<float, 6, 6>& At_A_in,
                                   const Matrix<float, 6, 1>& At_b_in,
                                         Matrix<float, 6, 1>* x_out) const {
  Cholesky<float, 6> cholesky(At_A_in);
  cholesky.SolveLinearEquations(At_b_in, x_out);
}

void DepthTrackingEngine::UpdateEstimateT(const Matrix<float, 6, 1>& x_in,
                                          const Matrix<float, 4, 4>& Tg_last_estimate_in,
                                                Matrix<float, 4, 4>* Tg_estimate_out) const {
  Matrix<float, 3, 4> T_inc = Matrix<float, 3, 4>::Identity();
  T_inc(0, 1) =  x_in(2, 0); T_inc(1, 2) =  x_in(0, 0); T_inc(2, 0) =  x_in(1, 0);
  T_inc(1, 0) = -x_in(2, 0); T_inc(2, 1) = -x_in(0, 0); T_inc(0, 2) = -x_in(1, 0);
  T_inc(0, 3) =  x_in(3, 0); T_inc(1, 3) =  x_in(4, 0); T_inc(2, 3) =  x_in(5, 0);
  Matrix<float, 3, 4> temp_Tg = T_inc * Tg_last_estimate_in;
  for (int c = 0; c <= 3; ++c)
    for (int r = 0; r <= 2; ++r)
      (*Tg_estimate_out)(r, c) = temp_Tg(r, c);
  (*Tg_estimate_out)(3, 0) = 0;
  (*Tg_estimate_out)(3, 1) = 0;
  (*Tg_estimate_out)(3, 2) = 0;
  (*Tg_estimate_out)(3, 3) = 1;
}

bool DepthTrackingEngine::IsConverged(const Matrix<float, 6, 1>& x_in) const {
  float norm2 = 0;
  for (int i = 0; i < 6; ++i)
    norm2 += x_in(i, 0) * x_in(i, 0);
  if (sqrt(norm2) / 6 < gDepthTrackConvergenceThreshold)
    return true;
  return false;
}
