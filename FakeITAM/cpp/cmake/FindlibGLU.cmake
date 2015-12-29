message(STATUS "Using bundled FindlibGLU.cmake...")
find_library(
  LIBGLU_LIBRARIES NAMES GLU
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
  )
