//
//  image_engine.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/5.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/image_engine.hpp"

#include <fstream>

#include "engines/library/calibrator.hpp"
#include "engines/library/image_utils.hpp"

using namespace std;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

const int ImageEngine::kMaxCacheFrameNum = 6;

ImageEngine::ImageEngine(const char* calib_filename)
    : current_frame_n_(-1), calibrator_(nullptr) {
  calibrator_ = new RGBDCalibrator(calib_filename);
}

ImageEngine::~ImageEngine() {
  if (calibrator_ != nullptr)
    delete calibrator_;
  calibrator_ = nullptr;
}

ImageFileReader::ImageFileReader(const char* calib_filename,
                                 const char* ppm_filename_format,
                                 const char* pgm_filename_format)
    : ImageEngine(calib_filename),
      ppm_format_(ppm_filename_format),
      pgm_format_(pgm_filename_format) {
  cached_rgb_files_ = new PNMImageFile*[kMaxCacheFrameNum];
  cached_depth_files_ = new PNMImageFile*[kMaxCacheFrameNum];
  cache_tags_ = new int[kMaxCacheFrameNum];
  for (int i = 0; i < kMaxCacheFrameNum; ++i) {
    cached_rgb_files_[i] = nullptr;
    cached_depth_files_[i] = nullptr;
    cache_tags_[i] = -1;
  }
}

ImageFileReader::~ImageFileReader() {
  for (int i = 0; i < kMaxCacheFrameNum; ++i) {
    if (cached_rgb_files_[i] != nullptr)
      delete cached_rgb_files_[i];
    if (cached_depth_files_[i] != nullptr)
      delete cached_depth_files_[i];
    cached_rgb_files_[i] = nullptr;
    cached_depth_files_[i] = nullptr;
  }
  delete [] cached_rgb_files_;
  delete [] cached_depth_files_;
  delete [] cache_tags_;
  cached_rgb_files_ = nullptr;
  cached_depth_files_ = nullptr;
  cache_tags_ = nullptr;
}

bool ImageFileReader::MoreFramesAvaliable() {
  int next_frame_num = current_frame_n_ + 1;
  if (load_frame_if_exists(next_frame_num))
    return true;
  else
    return false;
}

void ImageFileReader::NextRGBDFrame(const ImageRGB8u** rgb_frame,
                                    const ImageMono16u** raw_disparity_map) {
  if (MoreFramesAvaliable() == false) {
    if (rgb_frame != nullptr)
      *rgb_frame = nullptr;
    if (raw_disparity_map != nullptr)
      *raw_disparity_map = nullptr;
    return;
  }
  int cache_index = (++current_frame_n_) % kMaxCacheFrameNum;
  if (rgb_frame != nullptr) {
    const void* temp_rgb_file = cached_rgb_files_[cache_index]->image_data();
    *rgb_frame = (const ImageRGB8u*)(temp_rgb_file);
  }
  if (raw_disparity_map != nullptr) {
    const void* temp_depth_file = cached_depth_files_[cache_index]->image_data();
    *raw_disparity_map = (const ImageMono16u*)(temp_depth_file);
  }
}

void ImageFileReader::CurrentRGBDFrame(const ImageRGB8u** rgb_frame,
                                       const ImageMono16u** raw_disparity_map) const {
  if (current_frame_n_ < 0) {
    if (rgb_frame != nullptr)
      *rgb_frame = nullptr;
    if (raw_disparity_map != nullptr)
      *raw_disparity_map = nullptr;
    return;
  }
  int cache_index = current_frame_n_ % kMaxCacheFrameNum;
  if (rgb_frame != nullptr) {
    const void* temp_rgb_file = cached_rgb_files_[cache_index]->image_data();
    *rgb_frame = (const ImageRGB8u*)(temp_rgb_file);
  }
  if (raw_disparity_map != nullptr) {
    const void* temp_depth_file = cached_depth_files_[cache_index]->image_data();
    *raw_disparity_map = (const ImageMono16u*)(temp_depth_file);
  }
}

bool ImageFileReader::load_frame_if_exists(int frame_num) {
  if (is_frame_cached(frame_num))
    return true;

  char* ppm_filename = new char[ppm_format_.size() + 10];
  char* pgm_filename = new char[ppm_format_.size() + 10];
  sprintf(ppm_filename, ppm_format_.c_str(), frame_num);
  sprintf(pgm_filename, pgm_format_.c_str(), frame_num);
  if (fstream(ppm_filename).good() == false || 
      fstream(pgm_filename).good() == false) {
    delete [] ppm_filename;
    delete [] pgm_filename;
    return false;
  }

  PNMImageFile* ppm_file = new PNMImageFile(ppm_filename);
  PNMImageFile::LoadPNMFile(ppm_filename, ppm_file);
  if (cached_rgb_files_[frame_num % kMaxCacheFrameNum] != nullptr)
    delete cached_rgb_files_[frame_num % kMaxCacheFrameNum];
  cached_rgb_files_[frame_num % kMaxCacheFrameNum] = ppm_file;

  PNMImageFile* pgm_file = new PNMImageFile(pgm_filename);
  PNMImageFile::LoadPNMFile(pgm_filename, pgm_file);
  if (cached_depth_files_[frame_num % kMaxCacheFrameNum] != nullptr)
    delete cached_depth_files_[frame_num % kMaxCacheFrameNum];
  cached_depth_files_[frame_num % kMaxCacheFrameNum] = pgm_file;

  cache_tags_[frame_num % kMaxCacheFrameNum] = frame_num;
  delete [] ppm_filename;
  delete [] pgm_filename;
  return true;
}
