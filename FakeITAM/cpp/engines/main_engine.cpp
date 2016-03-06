//
//  main_engine.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/21.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/main_engine.hpp"

#include <cstdarg>
#include <ctime>
#include <stdexcept>

#include "global_config.hpp"
#include "engines/depth_tracking_engine.hpp"
#include "engines/image_engine.hpp"
#include "engines/log_engine.hpp"
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

MainEngine::MainEngine(const char* calib_filename, ...) : cycles_(-1) {
  LOG->WriteLine(I, "INITIALIZING Main Engine ...");

  va_list args;
  va_start(args, calib_filename);
  if (gInitializationFlags & USE_LIVE_DATA) {
    /* TODO Live data from RGB-D Camera */
  } else {
    const char* ppm_filename_format = va_arg(args, const char*);
    const char* pgm_filename_format = va_arg(args, const char*);
    LOG->WriteLine(I, "INITIALIZING Image Engine ...");
    image_engine_ = new ImageFileReader(calib_filename, ppm_filename_format, pgm_filename_format);
  }
  va_end(args);

  view_manager_ = new ViewManager(*image_engine_->calibrator(), gViewKernalSize);
  view_size_.x = image_engine_->calibrator()->depth_width();
  view_size_.y = image_engine_->calibrator()->depth_height();
  view_ = new View(nullptr, nullptr, view_size_, image_engine_->calibrator()->GetDepthIntrinsicVector());
  world_scene_ = new Scene(view_size_, gBlockHashOrderedArraySize, gBlockHashExcessListSize,
                           gBlockHashOrderedArraySize - 1, gBlockHashLocalNum, gVoxelBlockSizeC);
  camera_pose_ = new CameraPose;
  point_cloud_ = new PointCloud(view_size_, gRenderMaxPointCloudAge, MEM_CPU);
  if ((gInitializationFlags & USE_DEPTH_TRACKING) &&
      (gInitializationFlags & USE_COLOR_TRACKING) == 0 &&
      (gInitializationFlags & USE_FEATURES_TRACKING) == 0) {
    LOG->WriteLine(I, "INITIALIZING Tracking Engine ...");
    tracking_engine_ = new DepthTrackingEngine;
  } else {
    /* TODO Other tracking methods */
  }

  LOG->WriteLine(I, "INITIALIZING Reconstruction Engine ...");
  reconstruction_engine_ = new ReconstructionEngine;

  LOG->WriteLine(I, "INITIALIZING Rendering Engine ...");
  rendering_engine_ = new RenderingEngine(view_size_);

  LOG->WriteLine(I, "START PROCESSING ...")->WriteLine();
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
  delete view_;
  delete world_scene_;
  delete camera_pose_;
  delete point_cloud_;
  view_manager_ = nullptr;
  view_ = nullptr;
  world_scene_ = nullptr;
  camera_pose_ = nullptr;
  point_cloud_ = nullptr;
}

void MainEngine::ProcessOneFrame() throw(std::runtime_error) {
  ++cycles_;
  LOG->Write(I)
     ->WriteLineF("\033[31;1;4m\t****** current cycle ##%d ******\033[0m", cycles_);

  clock_t t, start = clock();
  float ms, total_ms = 0;

  /* Step 1: Read input "Frames" */
  LOG->WriteF(I, "loading RGB and depth frame ##%d ... ... ... ", image_engine_->current_frame_n() + 1);
  if (image_engine_->MoreFramesAvaliable() == false)
    throw runtime_error("Fatal Error: No more frames!");
  const ImageRGB8u* rgb_frame;
  const ImageMono16u* raw_disparity_map;
  this->image_engine_->NextRGBDFrame(&rgb_frame, &raw_disparity_map);
  //this->image_engine_->set_current_frame_n(0); this->image_engine_->CurrentRGBDFrame(&rgb_frame, &raw_disparity_map);
  LOG->WriteLineF("\033[31;1m\t%.2fms\033[0m", ms=((t=clock()-start)*1000.0/CLOCKS_PER_SEC));
  total_ms += ms;

  /* Step 2: Construct current "View" */
  LOG->Write(I, "constructing the current view ... ... ... ");
  view_->SetRGBDFrame(rgb_frame, raw_disparity_map);
  this->view_manager_->UpdateView(view_);
  LOG->WriteLineF("\033[31;1m\t%.2fms\033[0m", ms=((t=clock()-start)*1000.0/CLOCKS_PER_SEC));
  total_ms += ms;

  /* Step 3: Track the camera "CameraPose" */
  if (cycles_ > 0) {  /* Do not track camera at the first cycle */
    if ((gInitializationFlags & USE_DEPTH_TRACKING) &&
        (gInitializationFlags & USE_COLOR_TRACKING) == 0 &&
        (gInitializationFlags & USE_FEATURES_TRACKING) == 0) {
      LOG->Write(I, "running depth tracking method ... ... ... ");
    } else {
      /* TODO Other tracking methods */
    }
    this->tracking_engine_->TrackCamera(*view_, *point_cloud_, *camera_pose_, camera_pose_);
    //this->tracking_engine_->TrackCamera(*view_, *point_cloud_, *camera_pose_, nullptr);
    LOG->WriteLineF("\033[31;1m\t%.2fms\033[0m", ms=((t=clock()-start)*1000.0/CLOCKS_PER_SEC));
    total_ms += ms;
  }
  LOG->WriteLine()->Write(D, "\tTg: ")->WriteLine(camera_pose_->m)->WriteLine();

  /* Step 4: Reconstruct the world "Scene" */
  LOG->Write(I, "reconstructing the world scene ... ... ... ");
  this->reconstruction_engine_->AllocateWorldSceneFromView(*view_, *camera_pose_, world_scene_);
  this->reconstruction_engine_->IntegrateVoxelsToWorldScene(*view_, *camera_pose_, world_scene_);
  LOG->WriteLineF("\033[31;1m\t%.2fms\033[0m", ms=((t=clock()-start)*1000.0/CLOCKS_PER_SEC));
  total_ms += ms;

  /* Step 5: Render the "PointCloud" for next time tracking */
  if (gInitializationFlags & USE_FORWARD_PROJECTION) {
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
      LOG->Write(I, "processing full raycast rendering ... ... ... ");
      this->rendering_engine_->FullRenderIcpMaps(*world_scene_, *view_, *camera_pose_, point_cloud_);
      point_cloud_->ResetAge();
    } else {
      LOG->Write(I, "processing forward projection ... ... ... ");
      this->rendering_engine_->ForwardProject(*world_scene_, *view_, *camera_pose_, *point_cloud_, point_cloud_);
      point_cloud_->IncreaseAge();
    }
  } else {
    LOG->Write(I, "processing full raycast rendering ... ... ... ");
    this->rendering_engine_->FullRenderIcpMaps(*world_scene_, *view_, *camera_pose_, point_cloud_);
    point_cloud_->ResetAge();
  }
  LOG->WriteLineF("\033[31;1m\t%.2fms\033[0m", ms=((t=clock()-start)*1000.0/CLOCKS_PER_SEC));
  total_ms += ms;

  LOG->Write(I)
     ->WriteLineF("\033[31;1;4m\t****** current cycle ##%d accomplished (%.2fms) ******\033[0m", cycles_, total_ms)
     ->WriteLine();
}
