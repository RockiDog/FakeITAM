message(STATUS "Using bundled FindOpenGL.cmake...")
find_library(
  OPENGL_LIBRARIES NAMES OpenGL
  PATHS 
  /Library/Frameworks
  /System/Library/Frameworks
  /System/Library/PrivateFrameworks
  )
