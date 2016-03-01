//
//  cholesky.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/25.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_UTILITIES_CHOLESKY_HPP_
#define FAKEITAM_CPP_UTILITIES_CHOLESKY_HPP_

#include "utilities/matrix.hpp"

namespace fakeitam {
namespace utility {

template<typename T, int SIZE>
class Cholesky {
 public:
  Cholesky(const Matrix<T, SIZE, SIZE>& mat) {
    matrix_ = new Matrix<T, SIZE, SIZE>(mat);
    decompose();
  }

  ~Cholesky() {
    delete matrix_;
    matrix_ = nullptr;
  }

  const Matrix<T, SIZE, SIZE>* matrix() const { return matrix_; }

  /* Calculate the Ax = b result */
  void SolveLinearEquations(const Matrix<T, SIZE, 1>& b_in, Matrix<T, SIZE, 1>* x_out);

 private:
  void decompose();

  Matrix<T, SIZE, SIZE>* matrix_;
};

/* TODO Test */
template<typename T, int SIZE>
inline void Cholesky<T, SIZE>::decompose() {
  if (matrix_ == nullptr)
    return;
  Matrix<T, SIZE, SIZE>& mat_out = *matrix_;
  for (int c = 0; c < SIZE; ++c) {
    T diagonal = mat_out(0, 0);
    for (int r = c; r < SIZE; ++r) {
      T val = mat_out(r, c);
      for (int k = 0; k < c; ++k)
        val = val - mat_out(c, k) * mat_out(k, r);
      if (r == c) {
        mat_out(r, c) = val;
        diagonal = val;
      } else {
        mat_out(r, c) = val;
        mat_out(c, r) = val / diagonal;
      }
    }
  }
}

/* TODO Test */
template<typename T, int SIZE>
inline void Cholesky<T, SIZE>::SolveLinearEquations(
    const Matrix<T, SIZE, 1>& b_in,
    Matrix<T, SIZE, 1>* x_out) {
  Matrix<T, SIZE, 1>& result = *x_out;
  for (int i = 0; i < SIZE; ++i) {
    result(i, 0) = b_in(i, 0);
    for (int j = 0; j < i; ++j) {
      result(i, 0) = result(i, 0) - result(j, 0) * (*matrix_)(i, j);
      result(i, 0) = result(i, 0) / (*matrix_)(i, i);
    }
  }
  for (int i = SIZE - 1; i >= 0; --i) {
    for (int j = SIZE - 1; j > i; --j)
      result(i, 0) = result(i, 0) - result(j, 0) * (*matrix_)(i, j);
  }
}

}
}

#endif  /* FAKEITAM_CPP_UTILITIES_CHOLESKY_HPP_ */
