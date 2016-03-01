//
//  block_hash_manager.hpp
//  FakeITAM
//
//  Created by Soap on 15/12/4.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_BLOCK_HASH_MANAGER_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_BLOCK_HASH_MANAGER_HPP_

#include "utilities/mem_block.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

enum BlockAllocation {
  NO_REQUIREMENT,
  REQUIRE_ALLOCATION_ARRAY,
  REQUIRE_ALLOCATION_EXCESS
};

enum BlockVisibility {
  INVISIBLE,
  VISIBLE_IN_LAST_FRAME,
  VISIBLE_LOCALLY,
  VISIBLE_SWAPPED_OUT
};

struct BlockHashEntry {
  BlockHashEntry() : excess_offset_next(0), offset_in_array(-2) {}

  /*
   * The block coordinates, in block
   */
  utility::Vector3i position;

  /*
   * Offset in the excess list
   * = 0 : no next child
   * > 0 : the next child's offset in the excess list
   */
  int excess_offset_next;

  /*
   * The actual offset in the voxel block array
   * >= 0 : voxel block is in VBA[offset] and is active
   * = -1 : voxel block is active but has been swapped out
   * < -1 : identifies an unallocated voxel block
   */
  int offset_in_array;
};

class VoxelBlockHashMap {
 public:
  VoxelBlockHashMap(int ordered_array_size, int excess_list_size, int hash_mask);
  ~VoxelBlockHashMap();

  int BlockHash(const utility::Vector3i& block_coordinates) const;

  utility::MemBlock<BlockHashEntry>* hash_list() { return hash_list_; }
  utility::MemBlock<BlockVisibility>* visibility_table() { return visibility_table_; }
  utility::MemBlock<BlockAllocation>* allocation_types() { return allocation_types_; }
  utility::MemBlock<int>* excess_list_lookup() { return excess_list_lookup_; }
  int last_excess_list_id() const { return last_excess_list_id_; }
  void set_last_excess_list_id(int v) { last_excess_list_id_ = v; }
  const BlockHashEntry& operator[](int id) const { return (*hash_list_)[id]; }
  BlockHashEntry& operator[](int id) { return (*hash_list_)[id]; }

  const int kEntryNum;
  const int kHashMask;

 private:
  utility::MemBlock<BlockHashEntry>* hash_list_;
  utility::MemBlock<BlockVisibility>* visibility_table_;
  utility::MemBlock<BlockAllocation>* allocation_types_;
  utility::MemBlock<int>* excess_list_lookup_;
  int last_excess_list_id_;
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_BLOCK_HASH_MANAGER_HPP_ */
