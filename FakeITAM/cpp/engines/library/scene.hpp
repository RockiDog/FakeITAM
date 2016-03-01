//
//  scene.hpp
//  FakeITAM
//
//  Created by Soap on 15/12/4.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_SCENE_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_SCENE_HPP_

#include <vector>

#include "engines/library/block_hash_manager.hpp"
#include "utilities/mem_block.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

class PointCloud;

struct Voxel {
  Voxel() : sdf(32767), weight(0) {}
  short sdf;
  int weight;
};

class Scene {
 public:
  Scene(const utility::Vector2i& view_size,
        int hash_ordered_array_size, int hash_excess_list_size,
        int hash_map_mask, int local_voxel_array_size,
        int voxel_num_in_a_block);
  ~Scene();

  const VoxelBlockHashMap* index() const { return index_; }
  const utility::MemBlock<Voxel>* local_voxel_array() const { return local_voxel_array_; }
  const utility::MemBlock<int>* local_array_lookup() const { return local_array_lookup_; }
  const std::vector<int>& visible_list() const { return visible_list_; }
  int last_local_array_id() const { return last_local_array_id_; }

 private:
  VoxelBlockHashMap* index_;
  utility::MemBlock<Voxel>* local_voxel_array_;
  utility::MemBlock<int>* local_array_lookup_;
  std::vector<int> visible_list_;
  int last_local_array_id_;

  Scene(const Scene&);
  Scene& operator=(const Scene&);

  friend class ReconstructionEngine;
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_SCENE_HPP_ */
