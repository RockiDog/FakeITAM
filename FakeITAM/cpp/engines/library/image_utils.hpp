//
//  image_utils.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/7.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_IMAGE_UTILS_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_IMAGE_UTILS_HPP_

#include "utilities/mem_block.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

template<typename T>
class RGBColor : private utility::Vector3<T> {
 public:
  RGBColor() {
    utility::Vector3<T>::r = 0;
    utility::Vector3<T>::g = 0;
    utility::Vector3<T>::b = 0;
  }

  RGBColor(T red, T green, T blue) {
    utility::Vector3<T>::r = red;
    utility::Vector3<T>::g = green;
    utility::Vector3<T>::b = blue;
  }

  const T& R() const { return utility::Vector3<T>::r; }
  const T& G() const { return utility::Vector3<T>::g; }
  const T& B() const { return utility::Vector3<T>::b; }

  void set(T red, T green, T blue) {
    utility::Vector3<T>::r = red;
    utility::Vector3<T>::g = green;
    utility::Vector3<T>::b = blue;
  }
};

template<typename T>
class RGBAColor : private utility::Vector4<T> {
 public:
  RGBAColor() {
    utility::Vector4<T>::r = 0;
    utility::Vector4<T>::g = 0;
    utility::Vector4<T>::b = 0;
    utility::Vector4<T>::a = 0;
  }

  RGBAColor(T red, T green, T blue, T alpha) {
    utility::Vector4<T>::r = red;
    utility::Vector4<T>::g = green;
    utility::Vector4<T>::b = blue;
    utility::Vector4<T>::a = alpha;
  }

  const T& R() const { return utility::Vector4<T>::r; }
  const T& G() const { return utility::Vector4<T>::g; }
  const T& B() const { return utility::Vector4<T>::b; }
  const T& A() const { return utility::Vector4<T>::a; }

  void set(T red, T green, T blue, T alpha) {
    utility::Vector4<T>::r = red;
    utility::Vector4<T>::g = green;
    utility::Vector4<T>::b = blue;
    utility::Vector4<T>::a = alpha;
  }
};

typedef RGBColor<unsigned char> RGBColor8u;
typedef RGBColor<unsigned short> RGBColor16u;
typedef RGBAColor<unsigned char> RGBAColor8u;
typedef RGBAColor<unsigned short> RGBAColor16u;
typedef utility::MemBlock<unsigned char> ImageMono8u;
typedef utility::MemBlock<unsigned short> ImageMono16u;
typedef utility::MemBlock<float> ImageMono32f;
typedef utility::MemBlock<double> ImageMono64f;
typedef utility::MemBlock<RGBColor8u> ImageRGB8u;
typedef utility::MemBlock<RGBColor16u> ImageRGB16u;
typedef utility::MemBlock<RGBAColor8u> ImageRGBA8u;
typedef utility::MemBlock<RGBAColor16u> ImageRGBA16u;
typedef utility::MemBlock<utility::Vector3f> NormalXYZ32f;
typedef utility::MemBlock<utility::Vector4f> NormalXYZW32f;
enum PNMMagicValue { P1, P2, P3, P4, P5, P6 };
enum PNMImageDataType { MONO_8U, MONO_16U, RGB_8U, RGB_16U, INVALID };

class PNMHeader {
 public:
  PNMMagicValue magic_value() const { return magic_value_; }
  PNMImageDataType image_data_type() const { return image_data_type_; }
  int image_width() const { return image_width_; }
  int image_height() const { return image_height_; }
  int max_value() const { return max_value_; }
  long header_byte_n() const { return header_byte_n_; }

  static void ReadPNMHeader(const char* filename, PNMHeader* header);

 private:
  PNMMagicValue magic_value_;
  PNMImageDataType image_data_type_;
  long header_byte_n_;
  int image_width_;
  int image_height_;
  int max_value_;

  PNMHeader() = default;
  PNMHeader(const PNMHeader&);
  PNMHeader& operator=(const PNMHeader&);

  friend class PNMImageFile;
  friend std::istream& operator>>(std::istream& is, PNMMagicValue& value);
};

class PNMImageFile {
 public:
  PNMImageFile(const char* filename);
  ~PNMImageFile();

  const PNMHeader* header() const { return header_; }
  const void* image_data() const { return image_data_; }

  static void LoadPNMFile(const char* filename, PNMImageFile* image_file);

 private:
  PNMHeader* header_;
  void* image_data_;

  PNMImageFile(const PNMImageFile&);
  PNMImageFile& operator=(const PNMImageFile&);
};

utility::Vector4f BilinearInterpolationWithHoles(
    const utility::MemBlock<utility::Vector4f>& src,
    const utility::Vector2f& pos,
    const utility::Vector2i& size);

utility::Vector4d BilinearInterpolationWithHoles(
    const utility::MemBlock<utility::Vector4d>& src,
    const utility::Vector2f& pos,
    const utility::Vector2i& size);

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_IMAGE_UTILS_HPP_ */
