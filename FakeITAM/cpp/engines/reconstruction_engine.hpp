//
//  reconstruction_engine.hpp
//  FakeITAM
//
//  Created by Soap on 15/12/4.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_RECONSTRUCTION_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_RECONSTRUCTION_ENGINE_HPP_

#include "engines/library/block_hash_manager.hpp"
#include "engines/library/scene.hpp"
#include "utilities/matrix.hpp"

namespace fakeitam {
namespace engine {

struct View;
struct CameraPose;

class ReconstructionEngine {
 public:
  ReconstructionEngine();
  virtual ~ReconstructionEngine();

  virtual void ResetWorldScene(Scene* scene_out);
  virtual void AllocateWorldSceneFromView(const View& view_in,
                                          const CameraPose& camera_pose_in,
                                                Scene* scene_out);
  virtual void IntegrateVoxelsToWorldScene(const View& view_in,
                                           const CameraPose& camera_pose_in,
                                                 Scene* scene_out);
  static float ShortToFloat(short v) { return (float)v / 32767; }
  static short FloatToShort(float v) { return (short)(v * 32767); }

 private:
  void UpdateHashEntriesAndBlockCache(const View& view_in,
                                      const VoxelBlockHashMap& hash_map_in,
                                      const utility::Vector4f& intrinsics_in,
                                      const utility::Matrix4f& Tg_in,
                                            int x, int y,
                                            utility::MemBlock<utility::Vector3i>* block_cache_out,
                                            utility::MemBlock<BlockHashEntry>* block_hash_list_out,
                                            utility::MemBlock<BlockVisibility>* visibility_table_out,
                                            utility::MemBlock<BlockAllocation>* allocation_type_out);
  void UpdateVoxelTsdfAndWeight(const utility::Vector2i& view_size_in,
                                const utility::Matrix3f& intrinsics_in,
                                const utility::Matrix4f& Ti_g_in,
                                const utility::Vector3f& camera_coordinates_in,
                                const utility::MemBlock<float>& depth_in,
                                const utility::Vector4f& point_g_in,
                                      int max_W, float mu,
                                      Voxel* voxel_out);
  bool IsBlockVisible(const View& view_in,
                      const utility::Vector2i& view_size_in,
                      const utility::Vector4f& intrinsics_in,
                      const utility::Matrix4f& Tg_in,
                      const utility::Vector3i& block_in);
  bool IsVoxelVisible(const utility::Vector4f& voxel,
                      const utility::Vector2i& view_size_in,
                      const utility::Vector4f& intrinsics_in,
                      const utility::Matrix4f& Tg_in);

  utility::MemBlock<utility::Vector3i>* voxel_block_cache_;

  ReconstructionEngine(const ReconstructionEngine&);
  ReconstructionEngine& operator=(const ReconstructionEngine&);
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_RECONSTRUCTION_ENGINE_HPP_ */
