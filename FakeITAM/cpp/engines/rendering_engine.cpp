//
//  rendering_engine.cpp
//  FakeITAM
//
//  Created by Soap on 15/12/11.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "rendering_engine.hpp"

#include <cmath>
#include <vector>

#include "global_config.hpp"
#include "engines/reconstruction_engine.hpp"
#include "engines/tracking_engine.hpp"
#include "engines/library/block_hash_manager.hpp"
#include "engines/library/camera_pose.hpp"
#include "engines/library/point_cloud.hpp"
#include "engines/library/scene.hpp"
#include "engines/library/view_manager.hpp"
#include "utilities/matrix.hpp"
#include "utilities/vector.hpp"

using namespace std;
using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

RenderingEngine::RenderingEngine(Vector2i view_size) {
  range_resolution_ = new Vector2i(ceil(view_size.x * 1.0 / gBoundBoxSubsample),
                                   ceil(view_size.y * 1.0 / gBoundBoxSubsample));
  ray_length_range_ = new MemBlock<Vector2f>(range_resolution_->x * range_resolution_->y, MEM_CPU);
}

RenderingEngine::~RenderingEngine() {
  delete range_resolution_;
  delete ray_length_range_;
  range_resolution_ = nullptr;
  ray_length_range_ = nullptr;
}

/* TODO Test */
void RenderingEngine::FullRenderIcpMaps(const Scene& scene_in,
                                        const View& view_in,
                                        const CameraPose& pose_in,
                                              PointCloud* pcl_out) {
  const vector<int>& visible_blocks = scene_in.visible_list();
  const Matrix4f& Tg = pose_in.m;
  MemBlock<Vector4f> intersections(view_in.size.x * view_in.size.y, MEM_CPU);
  MemBlock<Vector4f> normals(view_in.size.x * view_in.size.y, MEM_CPU);

  FindBoundingBoxes(scene_in, view_in, Tg, visible_blocks, ray_length_range_);
  FullRaycast(scene_in, view_in, Tg, *range_resolution_, *ray_length_range_, &intersections);
  ComputeNormals(intersections, view_in.size, &normals);  /* TODO Compute angles */
  pcl_out->locations()->CopyBytesFrom(intersections.GetData(), intersections.byte_capacity());
  pcl_out->normals()->CopyBytesFrom(normals.GetData(), normals.byte_capacity());
  pcl_out->set_camera_pose(pose_in);
}

/* TODO Test */
void RenderingEngine::ForwardProject(const Scene& scene_in,
                                     const View& view_in,
                                     const CameraPose& pose_in,
                                     const PointCloud& pcl_in,
                                           PointCloud* pcl_out) {
  const vector<int>& visible_blocks = scene_in.visible_list();
  const Vector2i& view_size = view_in.size;
  const Matrix4f& Tg = pose_in.m;
  const MemBlock<Vector4f>& points = *pcl_in.locations();
  const MemBlock<float>& depth_map = *view_in.depth_map;
  PointCloud pcl(view_size, gRenderMaxPointCloudAge, MEM_CPU);

  const float fx = view_in.intrinsics.x;
  const float fy = view_in.intrinsics.y;
  const float cx = view_in.intrinsics.z;
  const float cy = view_in.intrinsics.w;

  FindBoundingBoxes(scene_in, view_in, Tg, visible_blocks, ray_length_range_);
  for (int y = 0; y < view_size.y; ++y) {
    for (int x = 0; x < view_size.x; ++x) {
      int id = x + y * view_size.x;
      Vector4f point_4d_g = points[id];
      Vector4f point_4d_l = Tg * point_4d_g;
      Vector2f point_2d(point_4d_l.x / point_4d_l.z * fx + cx, point_4d_l.y / point_4d_l.z * fy + cy);
      Vector2i pixel(round(point_2d.x), round(point_2d.y));
      if (pixel.x < 0 || pixel.x >= view_size.x || pixel.y < 0 || pixel.y >= view_size.y)
        point_4d_g.w = -1;
      int new_id = pixel.x + pixel.y * view_size.x;
      (*pcl.locations())[new_id] = point_4d_g;
    }
  }

  vector<int> missed_points_id;
  for (int y = 0; y < view_size.y; ++y) {
    for (int x = 0; x < view_size.x; ++x) {
      int id = x + y * view_size.x;
      const Vector4f& point = (*pcl.locations())[id];
      const Vector2f& range = (*ray_length_range_)[id];
      /* XXX Different with InfiniTAM */
      if (point.w <= 0 && range.x < range.y && depth_map[id] >= 0)
        missed_points_id.push_back(id);
    }
  }

  /* Raycast the missed points */
  Vector4f start_g, end_g;
  for (auto it = missed_points_id.begin(); it != missed_points_id.end(); ++it) {
    int id = *it;
    int x = id % view_size.x;
    int y = id / view_size.x;
    int range_x = x / gBoundBoxSubsample;
    int range_y = y / gBoundBoxSubsample;
    int range_id = range_x + range_y * range_resolution_->x;
    const Vector2f& range = (*ray_length_range_)[range_id];
    start_g.x = range.x * (x - cx) / fx;
    start_g.y = range.x * (y - cy) / fy;
    start_g.z = range.x;
    start_g.w = 1;
    start_g = Tg * start_g;
    end_g.x = range.y * (x - cx) / fx;
    end_g.y = range.y * (y - cy) / fy;
    end_g.z = range.y;
    end_g.w = 1;
    end_g = Tg * end_g;
    Vector4f& intersection = (*pcl.locations())[id];
    CastRay(scene_in, start_g, end_g, x, y, &intersection);
  }

  ComputeNormals(*pcl.locations(), view_size, pcl.normals());
  pcl_out->locations()->CopyBytesFrom(pcl.locations()->GetData(), pcl.locations()->byte_capacity());
  pcl_out->normals()->CopyBytesFrom(pcl.normals()->GetData(), pcl.normals()->byte_capacity());
}

/* TODO Test */
void RenderingEngine::FindBoundingBoxes(const Scene& scene_in,
                                        const View& view_in,
                                        const Matrix4f& Tg_in,
                                        const vector<int>& visible_blocks,
                                              MemBlock<Vector2f>* ray_length_range_out) {
  const VoxelBlockHashMap& index = *(scene_in.index());
  for (int i = 0; i < ray_length_range_out->element_n(); ++i) {
    (*ray_length_range_out)[i].x = gRaycastRangeZMax;
    (*ray_length_range_out)[i].y = gRaycastRangeZMin;
  }

  /* For each block */
  vector<BoundingBox> fragments;
  int total_fragment_num = 0;
  for (auto it = visible_blocks.begin(); it != visible_blocks.end(); ++it) {
    int visible_entry_id = *it;
    BlockHashEntry entry = index[visible_entry_id];
    if (entry.offset_in_array < 0)
      continue;
    
    /* Project one block */
    BoundingBox box;
    if (GetBoundingBox(entry.position, view_in, *range_resolution_, Tg_in, &box) == false)
      continue;
    
    /* Fragmentation */
    Vector2i box_size {box.lower_right.x - box.upper_left.x + 1,
                       box.lower_right.y - box.upper_left.y + 1};
    int fragment_x = ceil(box_size.x * 1.0 / gBoundBoxFragmentSizeL);
    int fragment_y = ceil(box_size.y * 1.0 / gBoundBoxFragmentSizeL);
    int fragment_num = fragment_x * fragment_y;
    if (total_fragment_num + fragment_num > gBoundBoxMaxFragmentNum)
      continue;
    total_fragment_num += fragment_num;
    for (int y = 0; y < fragment_y; ++y) {
      for (int x = 0; x < fragment_x; ++x) {
        Vector2i ul, lr;
        ul = box.upper_left + Vector2i(x, y) * gBoundBoxFragmentSizeL;
        lr = box.upper_left + Vector2i(x + 1, y + 1) * gBoundBoxFragmentSizeL;
        if (x == fragment_x - 1) lr.x = box.lower_right.x;
        if (y == fragment_y - 1) lr.y = box.lower_right.y;
        fragments.push_back(BoundingBox(ul, lr, box.depth_range));
      }
    }
  }

  for (auto it = fragments.begin(); it != fragments.end(); ++it) {
    const Vector2f& range = it->depth_range;
    for (int y = it->upper_left.y; y <= it->lower_right.y; ++y) {
      for (int x = it->upper_left.x; x <= it->lower_right.x; ++x) {
        int id = x + y * range_resolution_->x;
        if (range.x < (*ray_length_range_out)[id].x)
          (*ray_length_range_out)[id].x = range.x;
        if (range.y > (*ray_length_range_out)[id].y)
          (*ray_length_range_out)[id].y = range.y;
      }
    }
  }
}

void RenderingEngine::FullRaycast(
    const Scene& scene_in,
    const View& view_in,
    const Matrix4f& Tg_in,
    const Vector2i& range_resolution_in,
    const MemBlock<Vector2f>& ray_length_range_in,
          MemBlock<Vector4f>* points_out) {
  const Vector4f& intrinsics = view_in.intrinsics;
  const float fx = intrinsics.x;
  const float fy = intrinsics.y;
  const float cx = intrinsics.z;
  const float cy = intrinsics.w;

  /* For each pixel, find intersection */
  Vector4f start_g, end_g;
  for (int y = 0; y < view_in.size.y; ++y) {
    for (int x = 0; x < view_in.size.x; ++x) {
      int range_x = x / gBoundBoxSubsample;
      int range_y = y / gBoundBoxSubsample;
      int range_id = range_x + range_y * range_resolution_in.x;
      const Vector2f& range = ray_length_range_in[range_id];
      start_g.x = range.x * (x - cx) / fx;
      start_g.y = range.x * (y - cy) / fy;
      start_g.z = range.x;
      start_g.w = 1;
      start_g = Tg_in * start_g;
      end_g.x = range.y * (x - cx) / fx;
      end_g.y = range.y * (y - cy) / fy;
      end_g.z = range.y;
      end_g.w = 1;
      end_g = Tg_in * end_g;
      Vector4f& intersection = (*points_out)[x + y * view_in.size.x];
      CastRay(scene_in, start_g, end_g, x, y, &intersection);
    }
  }
}

/* TODO Test */
void RenderingEngine::ComputeNormals(const MemBlock<Vector4f>& points_in,
                                     const Vector2i& view_size_in,
                                           MemBlock<Vector4f>* normals_out) {
  for (int y = 0; y < view_size_in.y; ++y) {
    for (int x = 0; x < view_size_in.x; ++x) {
      int id = x + y * view_size_in.x;
      if (x < gViewKernalSize / 2 || x >= view_size_in.x - gViewKernalSize / 2 ||
          y < gViewKernalSize / 2 || y >= view_size_in.y - gViewKernalSize / 2) {
        (*normals_out)[id].x = 0;
        (*normals_out)[id].y = 0;
        (*normals_out)[id].z = 0;
        (*normals_out)[id].w = -1;
      } else {
        /* Cross product */
        Vector4f up = points_in[x + (y - 1) * view_size_in.x];
        Vector4f down = points_in[x + (y + 1) * view_size_in.x];
        Vector4f left = points_in[x - 1 + y * view_size_in.x];
        Vector4f right = points_in[x + 1 + y * view_size_in.x];
        Vector3f up_down = (down - up).ProjectTo3d();
        Vector3f left_right = (right - left).ProjectTo3d();
        (*normals_out)[id].x = up_down.y * left_right.z - up_down.z * left_right.y;
        (*normals_out)[id].y = up_down.z * left_right.x - up_down.x * left_right.z;
        (*normals_out)[id].z = up_down.x * left_right.y - up_down.y * left_right.x;
        if ((*normals_out)[id].x <= 0 && (*normals_out)[id].y <= 0 && (*normals_out)[id].z <=0) {
          (*normals_out)[id].w = -1;
        } else {
          (*normals_out)[id].w = 0;
          (*normals_out)[id] = (*normals_out)[id] / (*normals_out)[id].GetNorm();
          (*normals_out)[id].w = 1;
        }
      }
    }
  }
}

void RenderingEngine::CastRay(const Scene& scene_in,
                              const Vector4f& start_g_in,
                              const Vector4f& end_g_in,
                                    int x, int y,
                                    Vector4f* point_out) {
  float total_length = (end_g_in - start_g_in).GetNorm();
  int length = 0;
  Vector3f point {start_g_in.x, start_g_in.y, start_g_in.z};
  Vector3f one_step = (start_g_in - end_g_in).ProjectTo3d();
  one_step = one_step / one_step.GetNorm();
  float tsdf = 1;
  while (length < total_length) {
    float step_length;
    if (ReadNearestTsdf(scene_in, point, &tsdf) == false) {
      /* Jump by block size */
      step_length = gVoxelBlockSizeL * gVoxelMetricSize;
    } else {
      if (tsdf <= gRaycastSmallTsdfMax && tsdf >= gRaycastSmallTsdfMin) {
        /* Small T-SDF, read interpolated T-SDF */
        ReadInterpolatedTsdf(scene_in, point, &tsdf);
        if (tsdf < gMathFloatEpsilon)
          /* Found zero level of T-SDF */
          break;
        else
          /* Jump by interpolated T-SDF value */
          step_length = tsdf * gTsdfBandWidthMu;
      } else {
        /* Jump by T-SDF value */
        step_length = tsdf * gTsdfBandWidthMu;
      }
    }
    length += step_length;
    point = point + one_step * step_length;
  }
  if (tsdf < gMathFloatEpsilon) {
    /* Found */
    point_out->x = point.x;
    point_out->y = point.y;
    point_out->z = point.z;
    point_out->w = 1;
  } else {
    *point_out = Vector4f(0, 0, 0, -1);
  }
}

/* TODO Test */
bool RenderingEngine::GetBoundingBox(const Vector3i& block_in,
                                     const View& view_in,
                                     const Vector2i& range_resolution_in,
                                     const Matrix4f& Tg_in,
                                           BoundingBox* bounding_box_out) {
  const float fx = view_in.intrinsics.x;
  const float fy = view_in.intrinsics.y;
  const float cx = view_in.intrinsics.z;
  const float cy = view_in.intrinsics.w;
  const float size_factor = gVoxelBlockSizeL * gVoxelMetricSize;
  const Matrix4f Ti_g = GetInverse(Tg_in);

  /* Project 8 corners */
  float left_most = view_in.size.x;
  float upper_most = view_in.size.y;
  float right_most = -1, lower_most = -1;
  float z_max = gRaycastRangeZMin;
  float z_min = gRaycastRangeZMax;
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        Vector4f corner(block_in.x + x, block_in.y + y, block_in.z + z, 0);
        corner = corner * size_factor;
        if (corner.z < gMathFloatEpsilon)
          continue;
        corner.w = 1;
        corner = Ti_g * corner;
        float u = (corner.x / corner.z * fx + cx) / gBoundBoxSubsample;
        float v = (corner.y / corner.z * fy + cy) / gBoundBoxSubsample;
        if (u < left_most) left_most = u;
        if (u > right_most) right_most = u;
        if (v < upper_most) upper_most = v;
        if (v > lower_most) lower_most = v;
        if (corner.z > z_max) z_max = corner.z;
        if (corner.z < z_min) z_min = corner.z;
      }
    }
  }

  Vector2i upper_left(left_most, upper_most);
  Vector2i lower_right(ceil(right_most), ceil(lower_most));
  if (upper_left.x < 0 || upper_left.x >= range_resolution_in.x)
    return false;
  if (upper_left.y < 0 || upper_left.y >= range_resolution_in.y)
    return false;
  if (lower_right.x < 0 || lower_right.x >= range_resolution_in.x)
    return false;
  if (lower_right.y < 0 || lower_right.y >= range_resolution_in.y)
    return false;
  if (z_max <= z_min)
    return false;

  bounding_box_out->upper_left = upper_left;
  bounding_box_out->lower_right = lower_right;
  bounding_box_out->depth_range = Vector2f(z_min, z_max);
  return true;
}

bool RenderingEngine::ReadNearestTsdf(const Scene& scene_in,
                                      const Vector3f& point_in,
                                            float* tsdf_out) {
  const float block_metric_size = gVoxelMetricSize * gVoxelBlockSizeL;
  const VoxelBlockHashMap& index = *(scene_in.index());
  const MemBlock<Voxel>& voxel_array = *(scene_in.local_voxel_array());

  Vector3i block(point_in.x / block_metric_size,
                 point_in.y / block_metric_size,
                 point_in.z / block_metric_size);
  int hash = index.BlockHash(block);
  BlockHashEntry entry = index[hash];
  while (true) {
    if (entry.position.x == block.x &&
        entry.position.y == block.y &&
        entry.position.z == block.z &&
        entry.offset_in_array >= 0)
      break;  /* Found */
    if (entry.excess_offset_next > 0) {
      hash = gBlockHashOrderedArraySize + entry.excess_offset_next - 1;
      entry = index[hash];
    } else {
      *tsdf_out = 1;
      return false;  /* Not found */
    }
  }
  const Voxel* voxel_block = &voxel_array[entry.offset_in_array];
  float offset_x = point_in.x/gVoxelMetricSize - entry.position.x* gVoxelBlockSizeL;
  float offset_y = point_in.y/gVoxelMetricSize - entry.position.y* gVoxelBlockSizeL;
  float offset_z = point_in.z/gVoxelMetricSize - entry.position.z* gVoxelBlockSizeL;
  int offset = round(offset_z + offset_y* gVoxelBlockSizeL + offset_x* gVoxelBlockSizeQ);
  *tsdf_out = ReconstructionEngine::ShortToFloat(voxel_block[offset].sdf);
  return true;
}

void RenderingEngine::ReadInterpolatedTsdf(const Scene& scene_in,
                                           const Vector3f& point_in,
                                                 float* tsdf_out) {
  const VoxelBlockHashMap& index = *(scene_in.index());
  const MemBlock<Voxel>& voxel_array = *(scene_in.local_voxel_array());
  TrilinearInterpolation(index, voxel_array, point_in, tsdf_out);
}

/* TODO Test */
void RenderingEngine::TrilinearInterpolation(const VoxelBlockHashMap& index_in,
                                             const MemBlock<Voxel>& src_in,
                                             const Vector3f& position_in,
                                                   float* value_out) {
  Vector3i c[8];
  c[0] = Vector3i(position_in.x / gVoxelMetricSize,
                  position_in.y / gVoxelMetricSize,
                  position_in.z / gVoxelMetricSize);
  c[1] = c[0] + Vector3i(0, 0, 1);
  c[2] = c[0] + Vector3i(0, 1, 0);
  c[3] = c[0] + Vector3i(0, 1, 1);
  c[4] = c[0] + Vector3i(1, 0, 0);
  c[5] = c[0] + Vector3i(1, 0, 1);
  c[6] = c[0] + Vector3i(1, 1, 0);
  c[7] = c[0] + Vector3i(1, 1, 1);

  float v[8];
  for (int i = 0; i < 8; ++i) {
    Vector3i block = c[i] / gVoxelBlockSizeL;
    int hash = index_in.BlockHash(block);
    BlockHashEntry entry = index_in[hash];
    while (true) {
      if (entry.position.x == block.x &&
          entry.position.y == block.y &&
          entry.position.z == block.z &&
          entry.offset_in_array >= 0) {
        const Voxel* voxel_block = &src_in[entry.offset_in_array];
        int offset = (c[i].z - block.z * gVoxelBlockSizeL) +
                     (c[i].y - block.y * gVoxelBlockSizeL) * gVoxelBlockSizeL +
                     (c[i].x - block.x * gVoxelBlockSizeL) * gVoxelBlockSizeQ;
        v[i] = ReconstructionEngine::ShortToFloat(voxel_block[offset].sdf);
        break;  /* Found */
      }
      if (entry.excess_offset_next > 0) {
        hash = gBlockHashOrderedArraySize + entry.excess_offset_next - 1;
        entry = index_in[hash];
      } else {
        v[i] = 1;
        break;  /* Not found */
      }
    }
  }

  /* Interpolation */
  float v00 = (position_in.x / gVoxelMetricSize - c[0].x) * v[4] +
              (c[4].x - position_in.x / gVoxelMetricSize) * v[0];
  float v01 = (position_in.x / gVoxelMetricSize - c[1].x) * v[5] +
              (c[5].x - position_in.x / gVoxelMetricSize) * v[1];
  float v10 = (position_in.x / gVoxelMetricSize - c[2].x) * v[6] +
              (c[6].x - position_in.x / gVoxelMetricSize) * v[2];
  float v11 = (position_in.x / gVoxelMetricSize - c[3].x) * v[7] +
              (c[7].x - position_in.x / gVoxelMetricSize) * v[3];
  float v0 = (position_in.y / gVoxelMetricSize - c[0].y) * v10 +
             (c[2].y - position_in.y / gVoxelMetricSize) * v00;
  float v1 = (position_in.y / gVoxelMetricSize - c[1].y) * v11 +
             (c[3].y - position_in.y / gVoxelMetricSize) * v01;
  *value_out = (position_in.z / gVoxelMetricSize - c[0].z) * v1 +
               (c[1].z - position_in.z / gVoxelMetricSize) * v0;
}
