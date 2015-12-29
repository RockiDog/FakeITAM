//
//  scene.cpp
//  FakeITAM
//
//  Created by Soap on 15/12/4.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/library/scene.hpp"

#include "global_config.hpp"
#include "engines/library/point_cloud.hpp"
#include "utilities/mem_block.hpp"

using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

Scene::Scene(const Vector2i& view_size,
             int hash_ordered_array_size, int hash_excess_list_size,
             int hash_map_mask, int local_voxel_array_size,
             int voxel_num_in_a_block)
    : last_local_array_id_(local_voxel_array_size - 1) {
  index_ = new VoxelBlockHashMap(hash_ordered_array_size, hash_excess_list_size, hash_map_mask);
  int voxel_array_size = voxel_num_in_a_block * local_voxel_array_size;
  local_voxel_array_ = new MemBlock<Voxel>(voxel_array_size, MEM_CPU);
  local_array_lookup_ = new MemBlock<int>(local_voxel_array_size, MEM_CPU);
  for (int i = 0; i < local_voxel_array_size; ++i)
    (*local_array_lookup_)[i] = i;
}

Scene::~Scene() {
  delete index_;
  delete local_voxel_array_;
  delete local_array_lookup_;
  index_ = nullptr;
  local_voxel_array_ = nullptr;
  local_array_lookup_ = nullptr;
}
