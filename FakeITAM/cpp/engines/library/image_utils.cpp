//
//  image_utils.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/7.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "engines/library/image_utils.hpp"

#include <fstream>
#include <istream>
#include <sstream>
#include <string>

#include "global_config.hpp"

using namespace std;
using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

istream& operator>>(istream& is, PNMMagicValue& value) {
  char temp_value[3] = {0};
  is >> temp_value;
  switch (temp_value[1]) {
    case '1': value = P1; break;
    case '2': value = P2; break;
    case '3': value = P3; break;
    case '4': value = P4; break;
    case '5': value = P5; break;
    case '6': value = P6; break;
    default: break;
  }
  return is;
}

void PNMHeader::ReadPNMHeader(const char* filename, PNMHeader* header) {
  ifstream ifs(filename);
  string line;

  /* Read magic value */ {
    do {
      getline(ifs, line);
    } while (line.front() == '#');
    istringstream iss(line);
    iss >> header->magic_value_;
  }

  /* Read image width & height */ {
    do {
      getline(ifs, line);
    } while (line.front() == '#');
    istringstream iss(line);
    iss >> header->image_width_ >> header->image_height_;
  }

  /* Read element value */
  if (header->magic_value_ != P1 && header->magic_value_ != P4) {
    do {
      getline(ifs, line);
    } while (line.front() == '#');
    istringstream iss(line);
    iss >> header->max_value_;
  }

  if (header->magic_value_ == P2 || header->magic_value_ == P5) {
    if (header->max_value_ <= 0)
      header->image_data_type_ = INVALID;
    else if (header->max_value_ <= 0xFF)
      header->image_data_type_ = MONO_8U;
    else
      header->image_data_type_ = MONO_16U;
  } else if (header->magic_value_ == P3 || header->magic_value_ == P6) {
    if (header->max_value_ <= 0)
      header->image_data_type_ = INVALID;
    else if (header->max_value_ <= 0xFF)
      header->image_data_type_ = RGB_8U;
    else
      header->image_data_type_ = RGB_16U;
  } else {
    header->image_data_type_ = INVALID;
  }

  header->header_byte_n_ = ifs.tellg();
  ifs.close();
}

PNMImageFile::PNMImageFile(const char* filename) : image_data_(nullptr) {
  header_ = new PNMHeader;
  PNMHeader::ReadPNMHeader(filename, header_);
}

PNMImageFile::~PNMImageFile() {
  if (image_data_ != nullptr) {
    switch (header_->image_data_type_) {
      case MONO_8U: {
        ImageMono8u* data = (ImageMono8u*)(image_data_);
        ImageMono8u::Deallocate(data);
      } break;
      
      case MONO_16U: {
        ImageMono16u* data = (ImageMono16u*)(image_data_);
        ImageMono16u::Deallocate(data);
      } break;
      
      case RGB_8U: {
        ImageRGB8u* data = (ImageRGB8u*)(image_data_);
        ImageRGB8u::Deallocate(data);
      } break;
      
      case RGB_16U: {
        ImageRGB16u* data = (ImageRGB16u*)(image_data_);
        ImageRGB16u::Deallocate(data);
      } break;
      
      case INVALID: break;
    }
  }
  delete header_;
  image_data_ = nullptr;
  header_ = nullptr;
}

void PNMImageFile::LoadPNMFile(const char* filename, PNMImageFile* image_file) {
  const PNMHeader* header = image_file->header_;
  long header_byte_n = image_file->header_->header_byte_n_;
  int pixel_n = header->image_width_ * header->image_height_;

  /* Read raw image data */
  void*& raw_data = image_file->image_data_;

  switch (header->image_data_type_) {
    case MONO_8U: {
      raw_data = new ImageMono8u;
      ImageMono8u* data = (ImageMono8u*)(raw_data);
      ImageMono8u::Allocate(data, pixel_n, MEM_CPU);
    } break;
    
    case MONO_16U: {
      raw_data = new ImageMono16u;
      ImageMono16u* data = (ImageMono16u*)(raw_data);
      ImageMono16u::Allocate(data, pixel_n, MEM_CPU);
    } break;
    
    case RGB_8U: {
      raw_data = new ImageRGB8u;
      ImageRGB8u* data = (ImageRGB8u*)(raw_data);
      ImageRGB8u::Allocate(data, pixel_n, MEM_CPU);
    } break;
    
    case RGB_16U: {
      raw_data = new ImageRGB16u;
      ImageRGB16u* data = (ImageRGB16u*)(raw_data);
      ImageRGB16u::Allocate(data, pixel_n, MEM_CPU);
    } break;
    
    case INVALID: break;
  }

  switch (header->magic_value_) {
    case P1: { raw_data = nullptr; }
    case P4: { raw_data = nullptr; }
    
    case P2: {  /* ASCII greymap */
      ifstream ifs(filename);
      ifs.seekg(header_byte_n);
      unsigned int greyscale;
      for (int i = 0; i < pixel_n; ++i) {
        ifs >> greyscale;
        if (header->image_data_type_ == MONO_8U) {
          ImageMono8u* data = (ImageMono8u*)(raw_data);
          (*data)[i] = greyscale;
        } else if (header->image_data_type_ == MONO_16U) {
          ImageMono16u* data = (ImageMono16u*)(raw_data);
          (*data)[i] = greyscale;
        }
      }
      ifs.close();
    } break;
    
    case P3: {  /* ASCII pixmap */
      ifstream ifs(filename);
      ifs.seekg(header_byte_n);
      unsigned int red, green, blue;
      for (int i = 0; i < pixel_n; ++i) {
        ifs >> red >> green >> blue;
        if (header->image_data_type_ == RGB_8U) {
          ImageRGB8u* data = (ImageRGB8u*)(raw_data);
          (*data)[i].set(red, green, blue);
        } else if (header->image_data_type_ == MONO_16U) {
          ImageRGB16u* data = (ImageRGB16u*)(raw_data);
          (*data)[i].set(red, green, blue);
        }
      }
      ifs.close();
    } break;
    
    case P5: {  /* binary greymap */
      ifstream ifs(filename, ifstream::binary);
      ifs.seekg(header_byte_n);
      if (header->image_data_type_ == MONO_8U) {
        ImageMono8u* data = (ImageMono8u*)(raw_data);
        data->CopyBytesFromStream(ifs, pixel_n);
      } else if (header->image_data_type_ == MONO_16U) {
        ImageMono16u* data = (ImageMono16u*)(raw_data);
        data->CopyBytesFromStream(ifs, pixel_n * 2);
        for (int i = 0; i < data->element_n(); ++i)
          (*data)[i] = ((*data)[i] << 8) ^ ((*data)[i] >> 8 & 0x00FF);
      }
      ifs.close();
    } break;
    
    case P6: {  /* binary pixmap */
      ifstream ifs(filename, ifstream::binary);
      ifs.seekg(header_byte_n);
      if (header->image_data_type_ == RGB_8U) {
        ImageRGB8u* data = (ImageRGB8u*)(raw_data);
        data->CopyBytesFromStream(ifs, pixel_n * 3);
      } else if (header->image_data_type_ == RGB_16U) {
        ImageRGB16u* data = (ImageRGB16u*)(raw_data);
        data->CopyBytesFromStream(ifs, pixel_n * 2 * 3);
      }
      ifs.close();
    } break;
  }
}

Vector4f fakeitam::engine::BilinearInterpolationWithHoles(const MemBlock<Vector4f>& src,
                                                          const Vector2f& pos,
                                                          const Vector2i& size) {
  int x = pos.x, y = pos.y;
  if (x >= size.x || x < 0 || y >= size.y || y < 0)
    return Vector4f(0, 0, 0, -1);


  const Vector4f& a = src[x + 0 + (y + 0) * size.x];
  const Vector4f& b = src[x + 1 + (y + 0) * size.x];
  const Vector4f& c = src[x + 0 + (y + 1) * size.x];
  const Vector4f& d = src[x + 1 + (y + 1) * size.x];
  if (a.w <= gMathFloatEpsilon || b.w <= gMathFloatEpsilon ||
      c.w <= gMathFloatEpsilon || d.w <= gMathFloatEpsilon)
    return Vector4f(0, 0, 0, -1);

  float dx = pos.x - x;
  float dy = pos.y - y;
  return Vector4f(a.x*(1-dx)*(1-dy)+b.x*dx*(1-dy)+c.x*(1-dx)*dy+d.x*dx*dy,
                  a.y*(1-dx)*(1-dy)+b.y*dx*(1-dy)+c.y*(1-dx)*dy+d.y*dx*dy,
                  a.z*(1-dx)*(1-dy)+b.z*dx*(1-dy)+c.z*(1-dx)*dy+d.z*dx*dy,
                  a.w*(1-dx)*(1-dy)+b.w*dx*(1-dy)+c.w*(1-dx)*dy+d.w*dx*dy);
}

Vector4d fakeitam::engine::BilinearInterpolationWithHoles(const MemBlock<Vector4d>& src,
                                                          const Vector2f& pos,
                                                          const Vector2i& size) {
  int x = pos.x, y = pos.y;
  if (x >= size.x || x < 0 || y >= size.y || y < 0)
    return Vector4d(0, 0, 0, -1);

  const Vector4d& a = src[x + 0 + (y + 0) * size.x];
  const Vector4d& b = src[x + 1 + (y + 0) * size.x];
  const Vector4d& c = src[x + 0 + (y + 1) * size.x];
  const Vector4d& d = src[x + 1 + (y + 1) * size.x];
  if (a.w <= gMathDoubleEpsilon || b.w <= gMathDoubleEpsilon ||
      c.w <= gMathDoubleEpsilon || d.w <= gMathDoubleEpsilon)
    return Vector4d(0, 0, 0, -1);

  double dx = pos.x - x;
  double dy = pos.y - y;
  return Vector4d(a.x*(1-dx)*(1-dy)+b.x*dx*(1-dy)+c.x*(1-dx)*dy+d.x*dx*dy,
                  a.y*(1-dx)*(1-dy)+b.y*dx*(1-dy)+c.y*(1-dx)*dy+d.y*dx*dy,
                  a.z*(1-dx)*(1-dy)+b.z*dx*(1-dy)+c.z*(1-dx)*dy+d.z*dx*dy,
                  a.w*(1-dx)*(1-dy)+b.w*dx*(1-dy)+c.w*(1-dx)*dy+d.w*dx*dy);
}
