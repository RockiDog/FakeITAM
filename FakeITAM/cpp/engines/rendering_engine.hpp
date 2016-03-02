//
//  rendering_engine.hpp
//  FakeITAM
//
//  Created by Soap on 15/12/11.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_RENDERING_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_RENDERING_ENGINE_HPP_

#include <vector>

#include "engines/library/image_utils.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace utility {

/* Forward declarations */
template<typename T, int ROW_N, int COL_N> class Matrix;
template<typename T> class MemBlock;
typedef Matrix<float, 4, 4> Matrix4f;

}

namespace engine {

/* Forward declarations */
struct CameraPose;
struct View;
struct Voxel;
class PointCloud;
class Scene;
class VoxelBlockHashMap;

struct BoundingBox {
  BoundingBox() = default;
  BoundingBox(const utility::Vector2i& ul,
              const utility::Vector2i& lr,
              const utility::Vector2f& depth)
      : upper_left(ul), lower_right(lr), depth_range(depth) {}

  utility::Vector2i upper_left;
  utility::Vector2i lower_right;
  utility::Vector2f depth_range;
};

class RenderingEngine {
 public:
  RenderingEngine(utility::Vector2i view_size);
  virtual ~RenderingEngine();

  ImageMono8u* tsdf_map;

  const utility::Vector2i* range_resolution() const { return range_resolution_; }
  const utility::MemBlock<utility::Vector2f>* ray_length_range() const { return ray_length_range_; }

  virtual void FullRenderIcpMaps(const Scene& scene_in,
                                 const View& view_in,
                                 const CameraPose& pose_in,
                                       PointCloud* pcl_out);
  virtual void ForwardProject(const Scene& scene_in,
                              const View& view_in,
                              const CameraPose& pose_in,
                              const PointCloud& pcl_in,
                                    PointCloud* pcl_out);

 protected:
  virtual void FindBoundingBoxes(const Scene& scene_in,
                                 const View& view_in,
                                 const utility::Matrix4f& Ti_g_in,
                                 const std::vector<int>& visible_blocks,
                                       utility::MemBlock<utility::Vector2f>* ray_length_range_out);

 private:
  void FullRaycast(const Scene& scene_in,
                   const View& view_in,
                   const utility::Matrix4f& Tg_in,
                   const utility::Vector2i& range_resolution_in,
                   const utility::MemBlock<utility::Vector2f>& ray_length_range_in,
                         utility::MemBlock<utility::Vector4f>* points_out);

  void ComputeNormals(const utility::MemBlock<utility::Vector4f>& points_in,
                      const utility::Vector2i& view_size_in,
                            utility::MemBlock<utility::Vector4f>* normals_out);

  void CastRay(const Scene& scene_in,
               const utility::Vector4f& start_g_in,
               const utility::Vector4f& end_g_in,
                     int x, int y,
                     utility::Vector4f* point_out,
               const utility::Vector4f& intrinsics_in = utility::Vector4f());

  bool GetBoundingBox(const utility::Vector3i& block_in,
                      const View& view_in,
                      const utility::Vector2i& range_resolution_in,
                      const utility::Matrix4f& Ti_g_in,
                            BoundingBox* bounding_box_out);

  bool ReadNearestTsdf(const Scene& scene_in, const utility::Vector3f& point_in, float* tsdf_out,
               const utility::Vector4f& intrinsics_in = utility::Vector4f());
  void ReadInterpolatedTsdf(const Scene& scene_in, const utility::Vector3f& point_in, float* tsdf_out);
  void TrilinearInterpolation(const VoxelBlockHashMap& index_in,
                              const utility::MemBlock<Voxel>& src_in,
                              const utility::Vector3f& position_in,
                                    float* value_out);

  utility::Vector2i* range_resolution_;
  utility::MemBlock<utility::Vector2f>* ray_length_range_;

  RenderingEngine(const RenderingEngine&);
  RenderingEngine& operator=(const RenderingEngine&);
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_RENDERING_ENGINE_HPP_ */
