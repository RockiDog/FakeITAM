//
//  matrix.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/3.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_UTILITIES_MATRIX_HPP_
#define FAKEITAM_CPP_UTILITIES_MATRIX_HPP_

#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <typeinfo>

#include "utilities/vector.hpp"

namespace fakeitam {
namespace utility {

template<typename T, int ROW_N, int COL_N>
class Matrix {
 public:
  Matrix() {
    for (int i = 0; i < COL_N; ++i)
      std::memset(this->v[i], 0, sizeof(T) * ROW_N);
  }

  Matrix(std::initializer_list<T> init_values) {
    for (int i = 0; i < COL_N; ++i)
      std::memset(this->v[i], 0, sizeof(T) * ROW_N);
    for (auto it = init_values.begin(); it != init_values.end(); ++it) {
      long i = it - init_values.begin();
      if (i >= ROW_N * COL_N)
        break;
      v[i % COL_N][i / COL_N] = *it;
    }
  }

  Matrix(const Matrix<T, ROW_N, COL_N>& other) {
    for (int i = 0; i < COL_N; ++i)
      std::memcpy(this->v[i], other.v[i], sizeof(T) * ROW_N);
  }

  Matrix<T, COL_N, ROW_N> GetTranspose() {
    Matrix<T, COL_N, ROW_N> m_out;
    const Matrix<T, ROW_N, COL_N>& m = *this;
    for (int row = 0; row < COL_N; ++row)
      for (int col = 0; col < ROW_N; ++col)
          m_out(row, col) = m(col, row);
    return m_out;
  }

  Matrix<T, ROW_N, COL_N>& operator=(const Matrix<T, ROW_N, COL_N>& other) {
    for (int i = 0; i < COL_N; ++i)
      std::memcpy(this->v[i], other.v[i], sizeof(T) * ROW_N);
    return *this;
  }

  Matrix<T, ROW_N, COL_N> operator+(const Matrix<T, ROW_N, COL_N>& other) const {
    Matrix<T, ROW_N, COL_N> result;
    for (int i = 0; i < COL_N; ++i) {
      for (int j = 0; j < ROW_N; ++j)
        result.v[i][j] = this->v[i][j] + other.v[i][j];
    }
    return result;
  }

  Matrix<T, ROW_N, COL_N> operator-(const Matrix<T, ROW_N, COL_N>& other) const {
    Matrix<T, ROW_N, COL_N> result;
    for (int i = 0; i < COL_N; ++i) {
      for (int j = 0; j < ROW_N; ++j)
        result.v[i][j] = this->v[i][j] - other.v[i][j];
    }
    return result;
  }

  template<int COL_N2>
  Matrix<T, ROW_N, COL_N2> operator*(const Matrix<T, COL_N, COL_N2>& m) const {
    Matrix<T, ROW_N, COL_N2> m_out;
    for (int col2 = 0; col2 < COL_N2; ++col2) {
      for (int col1 = 0; col1 < COL_N; ++col1) {
        int& row2 = col1;
        for (int row1 = 0; row1 < ROW_N; ++row1) {
          int& row = row1;
          int& col = col2;
          m_out(row, col) += v[col1][row1] * m(row2, col2);
        }
      }
    }
    return m_out;
  }

  Matrix<T, ROW_N, COL_N> operator*(const T& operand) const {
    Matrix<T, ROW_N, COL_N> result;
    for (int i = 0; i < COL_N; ++i) {
      for (int j = 0; j < ROW_N; ++j)
        result.v[i][j] = this->v[i][j] * operand;
    }
    return result;
  }

  Matrix<T, ROW_N, COL_N> operator/(const T& operand) const {
    Matrix<T, ROW_N, COL_N> result;
    for (int i = 0; i < COL_N; ++i) {
      for (int j = 0; j < ROW_N; ++j)
        result.v[i][j] = this->v[i][j] / operand;
    }
    return result;
  }

  T& operator()(int row, int col) { return v[col][row]; }
  const T& operator()(int row, int col) const { return v[col][row]; }

  static Matrix<T, ROW_N, COL_N> Identity() {
    Matrix<T, ROW_N, COL_N> I;
    int size = ROW_N < COL_N ? ROW_N : COL_N;
    for (int i = 0; i < size; ++i)
      I.v[i][i] = 1;
    return I;
  }

 private:
  T v[COL_N][ROW_N];

  friend std::ostream& operator<<(std::ostream& os, const Matrix<T, ROW_N, COL_N>& m) {
    os << "\n";
    for (int i = 0; i < ROW_N; ++i) {
      for (int j = 0; j < COL_N; ++j)
        os << std::setw(4) << m(i, j) << " ";
      os << "\n";
    }
    os << std::endl;
    return os;
  }
};

template<typename T>
Vector2<T> operator*(const Matrix<T, 2, 2>& operand1, const Vector2<T>& operand2) {
  Vector2<T> result {0, 0};
  result.x += operand1(0, 0)*operand2.x + operand1(0, 1)*operand2.y;
  result.y += operand1(1, 0)*operand2.x + operand1(1, 1)*operand2.y;
  return result;
}

template<typename T>
Vector3<T> operator*(const Matrix<T, 3, 3>& operand1, const Vector3<T>& operand2) {
  Vector3<T> result {0, 0, 0};
  result.x = operand1(0, 0)*operand2.x + operand1(0, 1)*operand2.y + operand1(0, 2)*operand2.z;
  result.y = operand1(1, 0)*operand2.x + operand1(1, 1)*operand2.y + operand1(1, 2)*operand2.z;
  result.z = operand1(2, 0)*operand2.x + operand1(2, 1)*operand2.y + operand1(2, 2)*operand2.z;
  return result;
}

template<typename T>
Vector4<T> operator*(const Matrix<T, 4, 4>& operand1, const Vector4<T>& operand2) {
  Vector4<T> result {0, 0, 0, 0};
  result.x = operand1(0, 0)*operand2.x + operand1(0, 1)*operand2.y +
             operand1(0, 2)*operand2.z + operand1(0, 3)*operand2.w;
  result.y = operand1(1, 0)*operand2.x + operand1(1, 1)*operand2.y +
             operand1(1, 2)*operand2.z + operand1(1, 3)*operand2.w;
  result.z = operand1(2, 0)*operand2.x + operand1(2, 1)*operand2.y +
             operand1(2, 2)*operand2.z + operand1(2, 3)*operand2.w;
  result.w = operand1(3, 0)*operand2.x + operand1(3, 1)*operand2.y +
             operand1(3, 2)*operand2.z + operand1(3, 3)*operand2.w;
  return result;
}

template<typename T>
Vector6<T> operator*(const Matrix<T, 6, 6>& operand1, const Vector6<T>& operand2) {
  Vector6<T> result {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 6; ++i) {
    result.v[0] += operand1(0, i) * operand2.v[i];
    result.v[1] += operand1(1, i) * operand2.v[i];
    result.v[2] += operand1(2, i) * operand2.v[i];
    result.v[3] += operand1(3, i) * operand2.v[i];
    result.v[4] += operand1(4, i) * operand2.v[i];
    result.v[5] += operand1(5, i) * operand2.v[i];
  }
  return result;
}

template<typename T>
Matrix<T, 2, 2> GetInverse(const Matrix<T, 2, 2>& m) {
  Matrix<T, 2, 2> m_out;
  if (m(0, 0) * m(1, 1) != m(0, 1) * m(1, 0)) {
    m_out(0, 0) = m(1, 1);
    m_out(1, 1) = m(0, 0);
    m_out(0, 1) = -m(0, 1);
    m_out(1, 0) = -m(1, 0);
    m_out = m_out / (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0));
  }
  return m_out;
}

template<typename T>
Matrix<T, 4, 4> GetInverse(const Matrix<T, 4, 4>& m) {
  Matrix<T, 2, 2> A {m(0, 0), m(0, 1), m(1, 0), m(1, 1)};
  Matrix<T, 2, 2> B {m(0, 2), m(0, 3), m(1, 2), m(1, 3)};
  Matrix<T, 2, 2> C {m(2, 0), m(2, 1), m(3, 0), m(3, 1)};
  Matrix<T, 2, 2> D {m(2, 2), m(2, 3), m(3, 2), m(3, 3)};
  Matrix<T, 2, 2> A_inv = GetInverse(A);
  Matrix<T, 2, 2> DCAB = GetInverse(D - C * A_inv * B);
  Matrix<T, 2, 2> AB = A_inv * B;
  Matrix<T, 2, 2> CA = C * A_inv;

  Matrix<T, 2, 2> m0, m1, m2, m3;
  m0 = A_inv + AB * DCAB * CA;
  m1 = AB * -1 * DCAB;
  m2 = DCAB * -1 * CA;
  m3 = DCAB;
  return Matrix<T, 4, 4> {m0(0, 0), m0(0, 1), m1(0, 0), m1(0, 1),
                          m0(1, 0), m0(1, 1), m1(1, 0), m1(1, 1),
                          m2(0, 0), m2(0, 1), m3(0, 0), m3(0, 1),
                          m2(1, 0), m2(1, 1), m3(1, 0), m3(1, 1)};
}

typedef Matrix<int, 3, 3> Matrix3i;
typedef Matrix<float, 3, 3> Matrix3f;
typedef Matrix<double, 3, 3> Matrix3d;

typedef Matrix<int, 4, 4> Matrix4i;
typedef Matrix<float, 4, 4> Matrix4f;
typedef Matrix<double, 4, 4> Matrix4d;

typedef Matrix<int, 3, 4> Matrix34i;
typedef Matrix<float, 3, 4> Matrix34f;
typedef Matrix<double, 3, 4> Matrix34d;

}
}

#endif  /* FAKEITAM_CPP_UTILITIES_MATRIX_H_ */
