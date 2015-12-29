//
//  vector.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/3.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_UTILITIES_VECTOR_HPP_
#define FAKEITAM_CPP_UTILITIES_VECTOR_HPP_

#include <cmath>

namespace fakeitam {
namespace utility {

template<typename T, int ROW_N, int COL_N> class Matrix;

template<typename T>
struct Vector2 {
  Vector2() = default;
  Vector2(T x, T y) : x(x), y(y) {}

  Vector2 operator+(const Vector2& other) const { return Vector2(x + other.x, y + other.y); }
  Vector2 operator-(const Vector2& other) const { return Vector2(x - other.x, y - other.y); }
  Vector2 operator*(const T& operand) const { return Vector2(x * operand, y * operand); }
  Vector2 operator/(const T& operand) const { return Vector2(x / operand, y / operand); }
  operator Matrix<T, 2, 1>() const { return Matrix<T, 2, 1> {x, y}; }
  T GetNorm2() const { return x * x + y * y; }
  T GetNorm() const { return sqrt(x * x + y * y); }

  union {
    struct { T x, y; };
    T v[2];
  };
};

template<typename T>
struct Vector3 {
  Vector3() = default;
  Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

  Vector3 operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
  Vector3 operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }
  Vector3 operator*(const T& operand) const { return Vector3(x * operand, y * operand, z * operand); }
  Vector3 operator/(const T& operand) const { return Vector3(x / operand, y / operand, z / operand); }
  operator Matrix<T, 3, 1>() const { return Matrix<T, 3, 1> {x, y, z}; }
  Vector2<T> ProjectTo2d() const { return Vector2<T> {x, y}; }
  T GetNorm2() const { return x * x + y * y + z * z; }
  T GetNorm() const { return sqrt(x * x + y * y + z * z); }

  union {
    struct { T x, y, z; };
    struct { T r, g, b; };
    T v[3];
  };
};

template<typename T>
struct Vector4 {
  Vector4() = default;
  Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}

  Vector4 operator+(const Vector4& other) const {
    return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
  }
  Vector4 operator-(const Vector4& other) const {
    return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
  }
  Vector4 operator*(const T& operand) const {
    return Vector4(x * operand, y * operand, z * operand, w * operand);
  }
  Vector4 operator/(const T& operand) const {
    return Vector4(x / operand, y / operand, z / operand, w / operand);
  }

  operator Matrix<T, 4, 1>() const { return Matrix<T, 4, 1> {x, y, z, w}; }
  Vector3<T> ProjectTo3d() const { return Vector3<T> {x, y, z}; }
  T GetNorm2() const { return x * x + y * y + z * z + w * w; }
  T GetNorm() const { return sqrt(x * x + y * y + z * z + w * w); }

  union {
    struct { T x, y, z, w; };
    struct { T r, g, b, a; };
    T v[4];
  };
};

template<typename T>
struct Vector6 {
  Vector6() = default;
  Vector6(T v0, T v1, T v2, T v3, T v4, T v5) {
    v[0] = v0; v[1] = v1; v[2] = v2;
    v[3] = v3; v[4] = v4; v[5] = v5;
  }
  Vector6(const T* v) {
    for (int i = 0; i < 6; ++i)
      this->v[i] = v[i];
  }

  Vector6 operator+(const Vector6& other) const {
    Vector6 result;
    for (int i = 0; i < 6; ++i)
      result.v[i] = v[i] + other.v[i];
    return result;
  }
  Vector6 operator-(const Vector6& other) const {
    Vector6 result;
    for (int i = 0; i < 6; ++i)
      result.v[i] = v[i] - other.v[i];
    return result;
  }
  Vector6 operator*(const T& operand) const {
    Vector6 result;
    for (int i = 0; i < 6; ++i)
      result.v[i] = v[i] * operand;
    return result;
  }
  Vector6 operator/(const T& operand) const {
    Vector6 result;
    for (int i = 0; i < 6; ++i)
      result.v[i] = v[i] / operand;
    return result;
  }

  operator Matrix<T, 6, 1>() const {
    return Matrix<T, 6, 1> {v[0], v[1], v[2], v[3], v[4], v[5]};
  }
  T GetNorm2() const {
    T norm2 = 0;
    for (int i = 0; i < 6; ++i)
      norm2 += v[i] * v[i];
  }
  T GetNorm() const {
    return sqrt(GetNorm2());
  }

  T v[6];
};

typedef Vector2<int> Vector2i;
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

typedef Vector3<int> Vector3i;
typedef Vector3<short> Vector3s;
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

typedef Vector4<int> Vector4i;
typedef Vector4<float> Vector4f;
typedef Vector4<double> Vector4d;

}
}

#endif  /* FAKEITAM_CPP_UTILITIES_VECTOR_H_ */
