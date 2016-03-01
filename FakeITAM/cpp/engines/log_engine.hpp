//
//  log_engine.hpp
//  FakeITAM
//
//  Created by Soap on 16/1/10.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LOG_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_LOG_ENGINE_HPP_

#define LOG fakeitam::engine::LogEngine::Instance()

#include <cstdio>
#include <cstdarg>

#include "global_config.hpp"
#include "utilities/matrix.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

enum LogLevel {
  D,  /* DBG - Debug Information */
  I,  /* INF - Information, default */
  W,  /* WRN - Warnings */
  E   /* ERR - Errors */
};

class LogEngine {
 public:
  LogLevel print_level() const { return print_level_; }
  void set_print_level(LogLevel level) { print_level_ = level; }

  LogEngine* SetLogLevel(LogLevel level);

  LogEngine* WriteF(const char* format, ...);
  LogEngine* WriteF(LogLevel level, const char* format, ...);
  LogEngine* Write(LogLevel level);

  LogEngine* WriteLineF(const char* format, ...);
  LogEngine* WriteLineF(LogLevel level, const char* format, ...);
  LogEngine* WriteLine(LogLevel level);
  LogEngine* WriteLine();

  template<typename T> LogEngine* Write(const T& v) {
    if ((config::gInitializationFlags & config::USE_DEBUG_MODE) || current_level_ >= print_level_)
      write(v);
    return this;
  }

  template<typename T> LogEngine* WriteLine(const T& v) {
    if ((config::gInitializationFlags & config::USE_DEBUG_MODE) || current_level_ >= print_level_) {
      write(v);
      write("\n");
    }
    return this;
  }

  template<typename T> LogEngine* Write(LogLevel level, const T& v) {
    current_level_ = level;
    if ((config::gInitializationFlags & config::USE_DEBUG_MODE) || current_level_ >= print_level_) {
      prefix();
      write(v);
    }
    return this;
  }

  template<typename T> LogEngine* WriteLine(LogLevel level, const T& v) {
    current_level_ = level;
    if ((config::gInitializationFlags & config::USE_DEBUG_MODE) || current_level_ >= print_level_) {
      prefix();
      write(v);
      write("\n");
    }
    return this;
  }

  static LogEngine* Instance() {
    if (instance_ == nullptr)
      instance_ = new LogEngine;
    return instance_;
  }

  static void Release() {
    if (instance_ != nullptr)
      delete instance_;
    instance_ = nullptr;
  }

 private:
  void write(const char* str) const;
  void write(const char* format, va_list& args) const;
  void write(char v) const;
  void write(short v) const;
  void write(int v) const;
  void write(long v) const;
  void write(long long v) const;
  void write(float v) const;
  void write(double v) const;
  void write(unsigned char v) const;
  void write(unsigned short v) const;
  void write(unsigned int v) const;
  void write(unsigned long v) const;
  void write(unsigned long long v) const;
  void write(const void* v) const;
  void write(const utility::Vector2i& v) const;
  void write(const utility::Vector2f& v) const;
  void write(const utility::Vector3i& v) const;
  void write(const utility::Vector3f& v) const;
  void write(const utility::Vector4f& v) const;
  void write(const utility::Vector6<float>& v) const;

  template<int ROW_N, int COL_N>
  void write(const utility::Matrix<float, ROW_N, COL_N>& m) const {
    printf("Matrix<float, %d, %d>\033[36;1m@%p\033[0m:\n", ROW_N, COL_N, &m);
    prefix();
    printf("+-");
    for (int i = 0; i < COL_N; ++i)
      printf(" - - - - - ");
    printf("-+\n");
    for (int i = 0; i < ROW_N; ++i) {
      prefix();
      printf("| ");
      for (int j = 0; j < COL_N; ++j)
        printf("%10.4f ", m(i, j));
      printf(" |\n");
    }
    prefix();
    printf("+-");
    for (int i = 0; i < COL_N; ++i)
      printf(" - - - - - ");
    printf("-+");
  }

  void prefix() const {
    switch (current_level_) {
      case D: printf("\033[43;37;1mDBG\033[0m - "); break;
      case I: printf("\033[42;37;1mINF\033[0m - "); break;
      case W: printf("\033[45;37;1mWRN\033[0m - "); break;
      case E: printf("\033[41;37;1mERR\033[0m - "); break;
    }
  }

  static LogEngine* instance_;
  LogLevel print_level_;
  LogLevel current_level_;

  LogEngine() : print_level_(I), current_level_(I) {}
  ~LogEngine() { Release(); }
};

}
}

#endif  // FAKEITAM_CPP_ENGINES_LOG_ENGINE_HPP_
