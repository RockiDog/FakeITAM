message(STATUS "Using bundled FindlibGL.cmake...")
find_library(
  LIBGL_LIBRARIES NAMES GL
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
  )
