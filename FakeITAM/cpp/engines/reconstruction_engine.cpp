//
//  reconstruction_engine.cpp
//  FakeITAM
//
//  Created by Soap on 15/12/4.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/reconstruction_engine.hpp"

#include <cmath>
#include <vector>

#include "global_config.hpp"
#include "engines/log_engine.hpp"
#include "engines/tracking_engine.hpp"
#include "engines/library/block_hash_manager.hpp"
#include "engines/library/camera_pose.hpp"
#include "engines/library/view_manager.hpp"
#include "utilities/mem_block.hpp"

using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

ReconstructionEngine::ReconstructionEngine() {
  voxel_block_cache_ = new MemBlock<Vector3i>(gBlockHashMapSize, MEM_CPU);
  tsdf_map = new ImageMono8u(100 * 640 * 480, MEM_CPU);
  pcl = new utility::MemBlock<utility::Vector3f>(100 * 640 * 480, MEM_CPU);
  pcl_cnt = 0;
  pcl2 = new utility::MemBlock<utility::Vector3f>(100 * 640 * 480, MEM_CPU);
  pcl_cnt2 = 0;
}

ReconstructionEngine::~ReconstructionEngine() {
  delete voxel_block_cache_;
  voxel_block_cache_ = nullptr;

  delete tsdf_map;
  delete pcl;
  delete pcl2;
  tsdf_map = nullptr;
  pcl = nullptr;
  pcl2 = nullptr;
}

/* TODO Tired, implement later */
void ReconstructionEngine::ResetWorldScene(Scene* scene_out) { }

/* TODO Test */
static int cnt = 0;
void ReconstructionEngine::AllocateWorldSceneFromView(const View& view_in,
                                                      const CameraPose& camera_pose_in,
                                                            Scene* scene_out) {
  tsdf_map->ResetData();

  const Vector2i& view_size = view_in.size;
  const Vector4f& intrinsics = view_in.intrinsics;
  const Matrix4f& Tg = camera_pose_in.m;
  const Matrix4f Ti_g = GetInverse(Tg);

  /* For every pixel check its visibility */
  VoxelBlockHashMap& hash_map = *scene_out->index_;
  MemBlock<BlockHashEntry>* block_hash_list = hash_map.hash_list();
  MemBlock<BlockAllocation>* allocation_types = hash_map.allocation_types();
  MemBlock<BlockVisibility>* visibility_table = hash_map.visibility_table();
  for (auto it = scene_out->visible_list_.begin(); it != scene_out->visible_list_.end(); ++it)
    (*visibility_table)[*it] = VISIBLE_IN_LAST_FRAME;
  for (int y = 0; y < view_size.y; ++y) {
    for (int x = 0; x < view_size.x; ++x) {
      UpdateHashEntriesAndBlockCache(view_in, hash_map, intrinsics, Tg, x, y,
                                     voxel_block_cache_, block_hash_list,
                                     visibility_table, allocation_types);
    }
  }

  /* Allocate */
  MemBlock<int>* voxel_array_lookup = scene_out->local_array_lookup_;
  MemBlock<int>* excess_list_lookup = hash_map.excess_list_lookup();
  for (int i = 0; i < gBlockHashMapSize; ++i) {
    const Vector3i& block = (*voxel_block_cache_)[i];
    switch ((*allocation_types)[i]) {
      case REQUIRE_ALLOCATION_ARRAY: {
        int last_voxel_array_id = scene_out->last_local_array_id_;
        if (last_voxel_array_id >= 0) {
          BlockHashEntry entry;
          entry.position = block;
          entry.offset_in_array = (*voxel_array_lookup)[last_voxel_array_id--];
          entry.excess_offset_next = 0;
          
          (*block_hash_list)[i] = entry;
          scene_out->last_local_array_id_ = last_voxel_array_id;
          (*visibility_table)[i] = VISIBLE_LOCALLY;
          (*allocation_types)[i] = NO_REQUIREMENT;
        }
      } break;
      case REQUIRE_ALLOCATION_EXCESS: {
        int last_voxel_array_id = scene_out->last_local_array_id_;
        int last_excess_list_id = hash_map.last_excess_list_id();
        if (last_voxel_array_id >= 0 && last_excess_list_id >= 0) {
          BlockHashEntry entry;
          entry.position = block;
          entry.offset_in_array = (*voxel_array_lookup)[last_voxel_array_id--];
          entry.excess_offset_next = 0;
          
          int offset = (*excess_list_lookup)[last_excess_list_id--];
          (*block_hash_list)[i].excess_offset_next = offset + 1;
          offset += gBlockHashOrderedArraySize;
          (*block_hash_list)[offset] = entry;
          
          scene_out->last_local_array_id_ = last_voxel_array_id;
          hash_map.set_last_excess_list_id(last_excess_list_id);
          (*visibility_table)[offset] = VISIBLE_LOCALLY;
          (*allocation_types)[i] = NO_REQUIREMENT;
        }
      } break;
      default: break;
    }
  }

  scene_out->visible_list_.clear();
  for (int i = 0; i < gBlockHashMapSize; ++i) {
    if ((*visibility_table)[i] == VISIBLE_IN_LAST_FRAME) {
      const Vector3i& block = (*voxel_block_cache_)[i];
      if (IsBlockVisible(view_in, view_in.size, intrinsics, Ti_g, block) == false)
        (*visibility_table)[i] = INVISIBLE;
    }
    if ((*visibility_table)[i] != INVISIBLE) {
      scene_out->visible_list_.push_back(i);
    }
  }
}

/* TODO Test */
void ReconstructionEngine::UpdateHashEntriesAndBlockCache(const View& view_in,
                                                          const VoxelBlockHashMap& hash_map_in,
                                                          const Vector4f& intrinsics_in,
                                                          const Matrix4f& Tg_in,
                                                                int x, int y,
                                                                MemBlock<Vector3i>* block_cache_out,
                                                                MemBlock<BlockHashEntry>* block_hash_list_out,
                                                                MemBlock<BlockVisibility>* visibility_table_out,
                                                                MemBlock<BlockAllocation>* allocation_type_out) {
  int pixel_id = y * view_in.size.x + x;
  float depth = (*view_in.depth_map)[pixel_id];
  if ((depth >= 0) == false)
    return;
  const float block_metric_size = gVoxelMetricSize * gVoxelBlockSizeL;
  Vector4f point_4d_l {depth * (x - intrinsics_in.z) / intrinsics_in.x,
                       depth * (y - intrinsics_in.w) / intrinsics_in.y,
                       depth, 0};
  Vector4f direction_l = point_4d_l / point_4d_l.GetNorm();
  Vector4f front_4d_l = point_4d_l - direction_l * gTsdfBandWidthMu;
  Vector4f back_4d_l = point_4d_l + direction_l * gTsdfBandWidthMu;
  Vector3f front_3d_g = (Tg_in * front_4d_l / block_metric_size).ProjectTo3d();
  Vector3f back_3d_g = (Tg_in * back_4d_l / block_metric_size).ProjectTo3d();
  int step_n = ceil((back_3d_g - front_3d_g).GetNorm() * 2);
  Vector3f step_3d_g = (back_3d_g - front_3d_g) / (step_n - 1);

  for (int i = 0; i < step_n; ++i) {
    Vector3i block {(int)front_3d_g.x, (int)front_3d_g.y, (int)front_3d_g.z};
    int hash = hash_map_in.BlockHash(block);
    BlockHashEntry entry = (*block_hash_list_out)[hash];
    bool is_block_found = false;
    if (entry.position.x == block.x &&
        entry.position.y == block.y &&
        entry.position.z == block.z &&
        entry.offset_in_array >= -1) {
      is_block_found = true;
      (*allocation_type_out)[hash] = NO_REQUIREMENT;
      if (entry.offset_in_array >= 0)
        (*visibility_table_out)[hash] = VISIBLE_LOCALLY;
      else  /* offset_in_array == -1 */
        (*visibility_table_out)[hash] = VISIBLE_SWAPPED_OUT;
    }
    
    bool is_in_excess_list = false;
    if (is_block_found == false && entry.offset_in_array >= -1) {
      /* Probably in excess list */
      while (entry.excess_offset_next > 0) {
        hash = gBlockHashOrderedArraySize + entry.excess_offset_next - 1;
        entry = (*block_hash_list_out)[hash];
        if (entry.position.x == block.x &&
            entry.position.y == block.y &&
            entry.position.z == block.z &&
            entry.offset_in_array >= -1) {
          is_block_found = true;
          (*allocation_type_out)[hash] = NO_REQUIREMENT;
          if (entry.offset_in_array >= 0)
            (*visibility_table_out)[hash] = VISIBLE_LOCALLY;
          else  /* offset_in_array == -1 */
            (*visibility_table_out)[hash] = VISIBLE_SWAPPED_OUT;
          break;
        }
      }
      is_in_excess_list = true;
    }
    
    /* Still not found, require allocation */
    if (is_block_found == false) {
      if (is_in_excess_list)  /* The ordered list is full */
        (*allocation_type_out)[hash] = REQUIRE_ALLOCATION_EXCESS;
      else                    /* Allocate in the ordered list */
        (*allocation_type_out)[hash] = REQUIRE_ALLOCATION_ARRAY;
      (*block_cache_out)[hash] = block;
    }
    front_3d_g = front_3d_g + step_3d_g;
  }
}

/* TODO Test */
void ReconstructionEngine::IntegrateVoxelsToWorldScene(const View& view_in,
                                                       const CameraPose& camera_pose_in,
                                                             Scene* scene_out) {
  cnt = 0;
  const MemBlock<float>& depth = *(view_in.depth_map);
  const Vector2i& view_size = view_in.size;
  const Vector4f& intrinsics = view_in.intrinsics;
  const Matrix4f& Tg = camera_pose_in.m;
  Matrix3f intrinsic_mat {intrinsics.x, 0, intrinsics.z,
                          0, intrinsics.y, intrinsics.w,
                          0, 0, 1};
  Matrix4f Ti_g = GetInverse(Tg);
  Vector3f camera_coordinates = camera_pose_in.t();

  const MemBlock<BlockHashEntry>& hash_list = *(scene_out->index_->hash_list());
  const std::vector<int>& visible_list = scene_out->visible_list_;
  MemBlock<Voxel>& local_voxel_array = *(scene_out->local_voxel_array_);
  for (auto it = visible_list.begin(); it != visible_list.end(); ++it) {
    int visible_id = *it;
    BlockHashEntry entry = hash_list[visible_id];
    const Vector3i& corner = entry.position;
    Voxel* voxel_block = &local_voxel_array[entry.offset_in_array * gVoxelBlockSizeC];
    for (int z = 0; z < gVoxelBlockSizeL; ++z) {
      for (int x = 0; x < gVoxelBlockSizeL; ++x) {
        for (int y = 0; y < gVoxelBlockSizeL; ++y) {
          int id = x + y * gVoxelBlockSizeL + z * gVoxelBlockSizeQ;
          Voxel& voxel = voxel_block[id];
          Vector3i voxel_g;
          voxel_g.x = corner.x * gVoxelBlockSizeL + x;
          voxel_g.y = corner.y * gVoxelBlockSizeL + y;
          voxel_g.z = corner.z * gVoxelBlockSizeL + z;
          Vector4f point_g {voxel_g.x * gVoxelMetricSize,
                            voxel_g.y * gVoxelMetricSize,
                            voxel_g.z * gVoxelMetricSize, 1};
          UpdateVoxelTsdfAndWeight(view_size, intrinsic_mat, Ti_g,
                                   camera_coordinates, depth, point_g,
                                   gTsdfMaxWeight, gTsdfBandWidthMu, &voxel);
        }
      }
    }
  }
  LOG->WriteLine()->WriteLine(E, cnt);
}

/* TODO Test */
bool ReconstructionEngine::IsBlockVisible(const View& view_in,
                                          const Vector2i& view_size_in,
                                          const Vector4f& intrinsics_in,
                                          const Matrix4f& Ti_g_in,
                                          const Vector3i& block_in) {
  const float size_factor = gVoxelBlockSizeL * gVoxelMetricSize;
  Vector4f voxel;
  for (int z = 0; z < 2; ++z)
    for (int y = 0; y < 2; ++y)
      for (int x = 0; x < 2; ++x) {
        voxel.x = (block_in.x + x) * size_factor;
        voxel.y = (block_in.y + y) * size_factor;
        voxel.z = (block_in.z + z) * size_factor;
        if (IsVoxelVisible(voxel, view_size_in, intrinsics_in, Ti_g_in))
          return true;
      }
  return false;
}

/* TODO Test */
bool ReconstructionEngine::IsVoxelVisible(const Vector4f& voxel_in,
                                          const Vector2i& view_size_in,
                                          const Vector4f& intrinsics_in,
                                          const Matrix4f& Ti_g_in) {
  const float fx = intrinsics_in.x;
  const float fy = intrinsics_in.y;
  const float cx = intrinsics_in.z;
  const float cy = intrinsics_in.w;
  Vector3f point_3d_l = (Ti_g_in * voxel_in).ProjectTo3d();
  if (point_3d_l.z <= 0)
    return false;
  Vector2f point_2d_l(point_3d_l.x / point_3d_l.z * fx + cx,
                      point_3d_l.y / point_3d_l.z * fy + cy);
  if (point_2d_l.x >= 0 && point_2d_l.x <= view_size_in.x &&
      point_2d_l.y >= 0 && point_2d_l.y <= view_size_in.y)
    return true;
  else
    return false;
}

void ReconstructionEngine::UpdateVoxelTsdfAndWeight(const Vector2i& view_size_in,
                                                    const Matrix3f& intrinsics_in,
                                                    const Matrix4f& Ti_g_in,
                                                    const Vector3f& camera_coordinates_in,
                                                    const MemBlock<float>& depth_in,
                                                    const Vector4f& point_g_in,
                                                          int max_W, float mu,
                                                          Voxel* voxel_out) {
  Vector3f point_3d_l = (Ti_g_in * point_g_in).ProjectTo3d();
  Vector2f point_2d = (intrinsics_in * point_3d_l).ProjectTo2d() / point_3d_l.z;

  Vector2i pixel(round(point_2d.x), round(point_2d.y));
  if ((pixel.x < 1) || (pixel.x > view_size_in.x - 2) ||
      (pixel.y < 1) || (pixel.y > view_size_in.y - 2))
    return;

  float depth = depth_in[pixel.x + pixel.y * view_size_in.x];
  if (depth <= 0)
    return;

  float fx = intrinsics_in(0, 0), fy = intrinsics_in(1, 1);
  float dx = intrinsics_in(0, 2), dy = intrinsics_in(1, 2);
  float lambda = Vector3f((pixel.x - dx)/fx, (pixel.y - dy)/fy, 1).GetNorm();
  float eta = depth - (camera_coordinates_in - point_g_in.ProjectTo3d()).GetNorm() / lambda;
  if (eta <= -mu)
    return;

  float old_F = ShortToFloat(voxel_out->sdf);
  float new_F = eta / mu;
  if (new_F > 1)
    new_F = 1;

  int old_W = voxel_out->weight;
  int new_W = 1;

  float new_tsdf = (old_F * old_W + new_F * new_W) / (old_W + new_W);
  voxel_out->sdf = FloatToShort(new_tsdf);
  voxel_out->weight = (old_W + new_W) > max_W ? max_W : (old_W + new_W);
  if (255 - fabs(new_tsdf) * 255 > (*tsdf_map)[pixel.x + pixel.y * view_size_in.x])
    (*tsdf_map)[pixel.x + pixel.y * view_size_in.x] = 255 - (fabs(new_tsdf) <= 1 ? fabs(new_tsdf) * 255 : 255);
  if (point_g_in.w > 0) {
    (*pcl)[pcl_cnt++] = point_g_in.ProjectTo3d() / gVoxelMetricSize;
    (*pcl2)[pcl_cnt2].x = 1 - (fabs(new_tsdf) <= 1 ? fabs(new_tsdf) : 1);
    (*pcl2)[pcl_cnt2].y = 1 - (fabs(new_tsdf) <= 1 ? fabs(new_tsdf) : 1);
    (*pcl2)[pcl_cnt2++].z = 1 - (fabs(new_tsdf) <= 1 ? fabs(new_tsdf) : 1);
  }
}
