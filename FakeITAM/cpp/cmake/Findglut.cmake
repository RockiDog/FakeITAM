message(STATUS "Using bundled Findglut.cmake...")
find_library(
  GLUT_LIBRARIES NAMES glut
  PATHS
  /Library/Frameworks
  /System/Library/Frameworks
  /System/Library/PrivateFrameworks
  )
