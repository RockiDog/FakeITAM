//
//  log_engine.cpp
//  FakeITAM
//
//  Created by Soap on 16/1/10.
//  Copyright © 2015年 Soap. All rights reserved.
//

#include "global_config.hpp"
#include "engines/log_engine.hpp"
#include "utilities/matrix.hpp"

using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

LogEngine* LogEngine::instance_ = nullptr;

void LogEngine::write(const char* str) const { printf("%s", str); }
void LogEngine::write(const char* format, va_list& args) const { vprintf(format, args); }
void LogEngine::write(char v) const { printf("%c", v); }
void LogEngine::write(short v) const { printf("%hd", v); }
void LogEngine::write(int v) const { printf("%d", v); }
void LogEngine::write(long v) const { printf("%ld", v); }
void LogEngine::write(long long v) const { printf("%lld", v); }
void LogEngine::write(float v) const { printf("%f", v); }
void LogEngine::write(double v) const { printf("%f", v); }
void LogEngine::write(unsigned char v) const { printf("%c", v); }
void LogEngine::write(unsigned short v) const { printf("%hu", v); } 
void LogEngine::write(unsigned int v) const { printf("%u", v); }
void LogEngine::write(unsigned long v) const { printf("%lu", v); }
void LogEngine::write(unsigned long long v) const { printf("%llu", v); }
void LogEngine::write(const void* v) const { printf("\033[36;1m@%p\033[0m", v); }
void LogEngine::write(const Vector2i& v) const {
  printf("Vector2<int>\033[36;1m@%p\033[0m:\t[ ", &v);
  for (int i = 0; i < 2; ++i)
    printf("%6d ", v.v[i]);
  printf("]");
}
void LogEngine::write(const Vector2f& v) const {
  printf("Vector2<int>\033[36;1m@%p\033[0m:\t[ ", &v);
  for (int i = 0; i < 2; ++i)
    printf("%10.4f ", v.v[i]);
  printf("]");
}
void LogEngine::write(const Vector3i& v) const {
  printf("Vector3<int>\033[36;1m@%p\033[0m:\t[ ", &v);
  for (int i = 0; i < 3; ++i)
    printf("%6d ", v.v[i]);
  printf("]");
}
void LogEngine::write(const Vector3f& v) const {
  printf("Vector3<float>\033[36;1m@%p\033[0m:\t[ ", &v);
  for (int i = 0; i < 3; ++i)
    printf("%10.4f ", v.v[i]);
  printf("]");
}
void LogEngine::write(const Vector4f& v) const {
  printf("Vector4<float>\033[36;1m@%p\033[0m:\t[ ", &v);
  for (int i = 0; i < 4; ++i)
    printf("%10.4f ", v.v[i]);
  printf("]");
}
void LogEngine::write(const Vector6<float>& v) const {
  printf("Vector6<float>\033[36;1m@%p\033[0m:\t[ ", &v);
  for (int i = 0; i < 6; ++i)
    printf("%10.4f ", v.v[i]);
  printf("]");
}

LogEngine* LogEngine::WriteF(const char* format, ...) {
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_) {
    va_list args;
    va_start(args, format);
    write(format, args);
    va_end(args);
  }
  return this; 
}

LogEngine* LogEngine::WriteF(LogLevel level, const char* format, ...) {
  current_level_ = level;
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_) {
    prefix();
    va_list args;
    va_start(args, format);
    write(format, args);
    va_end(args);
  }
  return this; 
}

LogEngine* LogEngine::Write(LogLevel level) {
  current_level_ = level;
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_)
    prefix();
  return this;
}

LogEngine* LogEngine::WriteLineF(const char* format, ...) {
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_) {
    va_list args;
    va_start(args, format);
    write(format, args);
    va_end(args);
    write("\n");
  }
  return this; 
}

LogEngine* LogEngine::WriteLineF(LogLevel level, const char* format, ...) {
  current_level_ = level;
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_) {
    prefix();
    va_list args;
    va_start(args, format);
    write(format, args);
    va_end(args);
    write("\n");
  }
  return this; 
}

LogEngine* LogEngine::WriteLine(LogLevel level) {
  current_level_ = level;
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_) {
    prefix();
    write("\n");
  }
  return this;
}

LogEngine* LogEngine::WriteLine() {
  if ((gInitializationFlags & USE_DEBUG_MODE) || current_level_ >= print_level_)
    write("\n");
  return this;
}
