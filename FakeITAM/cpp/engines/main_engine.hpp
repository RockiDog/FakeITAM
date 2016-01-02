//
//  main_engine.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/21.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_MAIN_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_MAIN_ENGINE_HPP_

#include "engines/library/image_utils.hpp"
#include "utilities/vector.hpp"

#include <stdexcept>

namespace fakeitam {
namespace engine {

struct CameraPose;
struct View;
class ImageEngine;
class PointCloud;
class ReconstructionEngine;
class RenderingEngine;
class Scene;
class TrackingEngine;
class ViewManager;

class MainEngine {
 public:
  MainEngine(const char* calib_filename, ...);
  ~MainEngine();

  const ImageEngine* GetImageEngine() const { return image_engine_; }
  const TrackingEngine* GetTrackingEngine() const { return tracking_engine_; }
  const ReconstructionEngine* GetReconstructionEngine() const { return reconstruction_engine_; }
  const RenderingEngine* GetRenderingEngine() const { return rendering_engine_; }

  const ViewManager* view_manager() const { return view_manager_; }
  const View* view() const { return view_; }
  const Scene* world_scene() const { return world_scene_; }
  const CameraPose* camera_pose() const { return camera_pose_; }
  const PointCloud* point_cloud() const { return point_cloud_; }

  unsigned char flags() const { return flags_; }
  utility::Vector2i view_size() const { return view_size_; }
  int cycles() const { return cycles_; }

  void ProcessOneFrame() throw(std::runtime_error);

 private:
  ImageEngine* image_engine_;
  TrackingEngine* tracking_engine_;
  ReconstructionEngine* reconstruction_engine_;
  RenderingEngine* rendering_engine_;

  ViewManager* view_manager_;
  View* view_;
  Scene* world_scene_;
  CameraPose* camera_pose_;
  PointCloud* point_cloud_;

  const unsigned char flags_;
  utility::Vector2i view_size_;
  int cycles_;

  MainEngine(const MainEngine&);
  MainEngine& operator=(const MainEngine&);
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_MAIN_ENGINE_HPP_ */
