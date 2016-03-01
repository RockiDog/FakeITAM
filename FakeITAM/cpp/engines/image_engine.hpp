//
//  image_engine.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/5.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_IMAGE_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_IMAGE_ENGINE_HPP_

#include <string>

#include "engines/library/image_utils.hpp"

namespace fakeitam {
namespace engine {

class RGBDCalibrator;

class ImageEngine {
 public:
  static const int kMaxCacheFrameNum;

  virtual ~ImageEngine();

  int current_frame_n() const { return current_frame_n_; }
  void set_current_frame_n(int frame_n) { current_frame_n_ = frame_n; }
  const RGBDCalibrator* calibrator() const { return calibrator_; }

  virtual bool MoreFramesAvaliable() = 0;
  virtual void NextRGBDFrame(const ImageRGB8u** rgb_frame,
                             const ImageMono16u** raw_disparity_map) = 0;
  virtual void CurrentRGBDFrame(const ImageRGB8u** rgb_frame,
                                const ImageMono16u** raw_disparity_map) const = 0;

 protected:
  ImageEngine(const char* calib_filename);

  int current_frame_n_;
  RGBDCalibrator* calibrator_;

 private:
  ImageEngine(const ImageEngine&);
  ImageEngine& operator=(const ImageEngine&);
};

class ImageFileReader : public ImageEngine {
 public:
  ImageFileReader(const char* calib_filename,
                  const char* ppm_filename_format,
                  const char* pgm_filename_format);
  virtual ~ImageFileReader();

  virtual bool MoreFramesAvaliable();
  virtual void NextRGBDFrame(const ImageRGB8u** rgb_frame,
                             const ImageMono16u** raw_disparity_map);
  virtual void CurrentRGBDFrame(const ImageRGB8u** rgb_frame,
                                const ImageMono16u** raw_disparity_map) const;

 private:
  bool load_frame_if_exists(int frame_num);
  bool is_frame_cached(int frame_num);

  PNMImageFile** cached_rgb_files_;
  PNMImageFile** cached_depth_files_;
  int* cache_tags_;

  const std::string ppm_format_;
  const std::string pgm_format_;
};

inline bool ImageFileReader::is_frame_cached(int frame_num) {
  if (frame_num < 0)
    return false;
  return (frame_num == cache_tags_[frame_num % kMaxCacheFrameNum]);
}

}
}

#endif  /* FAKEITAM_CPP_ENGINES_IMAGE_ENGINE_HPP_ */
