message(STATUS "Using bundled Findlibglut.cmake...")
find_library(
  LIBGLUT_LIBRARIES NAMES glut
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
  )
