//
//  block_hash_manager.cpp
//  FakeITAM
//
//  Created by Soap on 15/12/4.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/library/block_hash_manager.hpp"

#include "global_config.hpp"
#include "utilities/mem_block.hpp"

using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

VoxelBlockHashMap::VoxelBlockHashMap(int ordered_array_size, int excess_list_size, int hash_mask)
    : kEntryNum(ordered_array_size + excess_list_size),
      kHashMask(hash_mask),
      last_excess_list_id_(excess_list_size - 1) {
  hash_list_ = new MemBlock<BlockHashEntry>(kEntryNum, MEM_CPU);
  visibility_table_ = new MemBlock<BlockVisibility>(kEntryNum, MEM_CPU);
  allocation_types_ = new MemBlock<BlockAllocation>(kEntryNum, MEM_CPU);
  excess_list_lookup_ = new MemBlock<int>(excess_list_size, MEM_CPU);
  visibility_table_->ResetData();
  for (int i = 0; i < excess_list_size; ++i)
    (*excess_list_lookup_)[i] = i;
}

VoxelBlockHashMap::~VoxelBlockHashMap() {
  delete hash_list_;
  delete visibility_table_;
  delete allocation_types_;
  delete excess_list_lookup_;
  hash_list_ = nullptr;
  visibility_table_ = nullptr;
  allocation_types_ = nullptr;
  excess_list_lookup_ = nullptr;
}

int VoxelBlockHashMap::BlockHash(const Vector3i& block_coordinates) const {
	return ((block_coordinates.x * 73856093) ^
          (block_coordinates.y * 19349669) ^
          (block_coordinates.z * 83492791)) &
          kHashMask;
}
