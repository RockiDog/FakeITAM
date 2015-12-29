//
//  main_engine.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/21.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/main_engine.hpp"

#include <cstdarg>
#include <iostream>
#include <stdexcept>

#include "global_config.hpp"
#include "engines/depth_tracking_engine.hpp"
#include "engines/image_engine.hpp"
#include "engines/reconstruction_engine.hpp"
#include "engines/rendering_engine.hpp"
#include "engines/tracking_engine.hpp"
#include "engines/library/camera_pose.hpp"
#include "engines/library/point_cloud.hpp"
#include "engines/library/view_manager.hpp"
#include "utilities/vector.hpp"

using namespace std;
using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

MainEngine::MainEngine(const char* calib_filename, ...) : flags_(gInitializationFlags), cycles_(-1) {
  if (flags_ & USE_DEBUG_MODE)
    cout << "\nINITIALIZING Main Engine ..." << endl;

  va_list args;
  va_start(args, calib_filename);
  if (flags_ & USE_LIVE_DATA) {
    /* TODO Live data from RGB-D Camera */
  } else {
    const char* ppm_filename_format = va_arg(args, const char*);
    const char* pgm_filename_format = va_arg(args, const char*);
    if (flags_ & USE_DEBUG_MODE)
      cout << "INITIALIZING Image Engine ..." << endl;
    image_engine_ = new ImageFileReader(calib_filename, ppm_filename_format, pgm_filename_format);
  }
  va_end(args);

  view_manager_ = new ViewManager(*image_engine_->calibrator(), gViewKernalSize);
  view_size_.x = image_engine_->calibrator()->depth_width();
  view_size_.y = image_engine_->calibrator()->depth_height();
  world_scene_ = new Scene(view_size_, gBlockHashOrderedArraySize, gBlockHashExcessListSize,
                           gBlockHashOrderedArraySize - 1, gBlockHashLocalNum, gVoxelBlockSizeC);
  camera_pose_ = new CameraPose;
  point_cloud_ = new PointCloud(view_size_, MEM_CPU);
  if ((flags_ & USE_DEPTH_TRACKING) &&
      (flags_ & USE_COLOR_TRACKING) == 0 &&
      (flags_ & USE_FEATURES_TRACKING) == 0) {
    if (flags_ & USE_DEBUG_MODE)
      cout << "INITIALIZING Tracking Engine ..." << endl;
    tracking_engine_ = new DepthTrackingEngine;
  }
  /* TODO Other tracking methods */

  if (flags_ & USE_DEBUG_MODE)
    cout << "INITIALIZING Reconstruction Engine ..." << endl;
  reconstruction_engine_ = new ReconstructionEngine;

  if (flags_ & USE_DEBUG_MODE)
    cout << "INITIALIZING Rendering Engine ..." << endl;
  rendering_engine_ = new RenderingEngine(view_size_);
  cout << "START PROCESSING ...\n" << endl;
}

MainEngine::~MainEngine() {
  delete image_engine_;
  delete tracking_engine_;
  delete reconstruction_engine_;
  delete rendering_engine_;
  image_engine_ = nullptr;
  tracking_engine_ = nullptr;
  reconstruction_engine_ = nullptr;
  rendering_engine_ = nullptr;

  delete view_manager_;
  delete world_scene_;
  delete camera_pose_;
  delete point_cloud_;
  view_manager_ = nullptr;
  world_scene_ = nullptr;
  camera_pose_ = nullptr;
  point_cloud_ = nullptr;
}

void MainEngine::ProcessOneFrame() throw(std::runtime_error) {
  ++cycles_;
  if (flags_ & USE_DEBUG_MODE)
    cout << " ****** current cycle ##" << cycles_ << endl;

  /* Step 1: Read input "Frames" */
  if (flags_ & USE_DEBUG_MODE)
    cout << "\tprocessing frame ##" << image_engine_->current_frame_n() + 1 << " ... ... ..." << flush;
  if (image_engine_->MoreFramesAvaliable() == false)
    throw runtime_error("Fatal Error: No more frames!");
  const ImageRGB8u* rgb_frame;
  const ImageMono16u* raw_disparity_map;
  this->image_engine_->NextRGBDFrame(&rgb_frame, &raw_disparity_map);
  if (flags_ & USE_DEBUG_MODE)
    cout << " end" << endl;

  /* Step 2: Construct current "View" */
  if (flags_ & USE_DEBUG_MODE)
    cout << "\tconstructing current view ... ... ..." << flush;
  View view(rgb_frame, raw_disparity_map, view_size_, image_engine_->calibrator()->GetDepthIntrinsicVector());
  this->view_manager_->UpdateView(&view);
  if (flags_ & USE_DEBUG_MODE)
    cout << " end" << endl;

  /* Step 3: Track the camera "CameraPose" */
  if (cycles_ > 0) {  /* Do not track camera at the first cycle */
    if (flags_ & USE_DEBUG_MODE) {
      if ((flags_ & USE_DEPTH_TRACKING) &&
          (flags_ & USE_COLOR_TRACKING) == 0 &&
          (flags_ & USE_FEATURES_TRACKING) == 0)
        cout << "\trunning depth tracking ... ... ..." << flush;
      /* TODO Other tracking methods */
    }
    this->tracking_engine_->TrackCamera(view, *point_cloud_, *camera_pose_, camera_pose_);
    if (flags_ & USE_DEBUG_MODE)
      cout << " end" << endl;
  }

  /* Step 4: Reconstruct the world "Scene" */
  if (flags_ & USE_DEBUG_MODE)
    cout << "\treconstructing world scene ... ... ..." << flush;
  this->reconstruction_engine_->AllocateWorldSceneFromView(view, *camera_pose_, world_scene_);
  this->reconstruction_engine_->IntegrateVoxelsToWorldScene(view, *camera_pose_, world_scene_);
  if (flags_ & USE_DEBUG_MODE)
    cout << " end" << endl;

  /* Step 5: Render the "PointCloud" for next time tracking */
  if (flags_ & USE_FORWARD_PROJECTION) {
    bool require_full_rendering = false;
    if (point_cloud_->age() == gRenderMaxPointCloudAge) {
      require_full_rendering = true;
    } else {
      Vector3f current_pose = camera_pose_->R().GetTranspose() * camera_pose_->t();
      Vector3f pcl_pose = point_cloud_->R().GetTranspose() * point_cloud_->t();
      float distance = (pcl_pose - current_pose).GetNorm2();
      require_full_rendering = distance > gRenderMaxCameraDistance2 ? true : false;
    }
    if (require_full_rendering) {
      if (flags_ & USE_DEBUG_MODE)
        cout << "\tprocessing full raycasting ... ... ..." << flush;
      this->rendering_engine_->FullRenderIcpMaps(*world_scene_, view, *camera_pose_, point_cloud_);
      point_cloud_->ResetAge();
    } else {
      if (flags_ & USE_DEBUG_MODE)
        cout << "\tprocessing forward projection ... ... ..." << flush;
      this->rendering_engine_->ForwardProject(*world_scene_, view, *camera_pose_, *point_cloud_, point_cloud_);
      point_cloud_->IncreaseAge();
    }
  } else {
    if (flags_ & USE_DEBUG_MODE)
      cout << "\tprocessing full raycasting ... ... ..." << flush;
    this->rendering_engine_->FullRenderIcpMaps(*world_scene_, view, *camera_pose_, point_cloud_);
    point_cloud_->ResetAge();
  }
  if (flags_ & USE_DEBUG_MODE)
    cout << " end" << endl;

  if (flags_ & USE_DEBUG_MODE)
    cout << " ****** current cycle ##" << cycles_ << " accomplished\n" << endl;
}
