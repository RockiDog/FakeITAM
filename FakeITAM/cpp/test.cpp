//
//  test.cpp
//  FakeITAM
//
//  Created by Soap on 15/11/23.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#elif __linux__
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <iostream>
#include <stdexcept>

#include "engines/image_engine.hpp"
#include "engines/main_engine.hpp"
#include "engines/library/calibrator.hpp"
#include "engines/library/view_manager.hpp"

using namespace std;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

namespace {

MainEngine* g_main_engine = nullptr;
const ImageRGB8u* g_rgb_image = nullptr;
const ImageMono32f* g_depth_image = nullptr;
ImageMono8u* g_greyscale_image = nullptr;
bool g_stalled = false;
float g_win_width = 0;
float g_win_height = 0;

GLsizei texture_n = 2;
GLuint textures[2];

}

void ConvertDepthToGreyscale(const ImageMono32f& depths, ImageMono8u* greyscales) {
  for (int i = 0; i < depths.element_n(); ++i) {
    float depth = depths[i];
    if (depth < 0)
      depth = 0;
    unsigned char greyscale = depth * 255;
    (*greyscales)[i] = greyscale;
  }
}

void DisplayFunc() {
  if (g_rgb_image == nullptr)
    return;

  /* Do the actual drawing */
  glClear(GL_COLOR_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glPushMatrix(); {
    /* Display RGB image */
    glBindTexture(GL_TEXTURE_2D, textures[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_win_width / 2, g_win_height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, g_rgb_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex2f(0, g_win_height);                /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width / 2, g_win_height);  /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width / 2, 0);             /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(0, 0);                           /* bottom-left */
    } glEnd();
    
    /* Display depth frame */
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, g_win_width / 2, g_win_height,
                 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g_greyscale_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex2f(g_win_width / 2, g_win_height);  /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width, g_win_height);      /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width, 0);                 /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(g_win_width / 2, 0);             /* bottom-left */
    } glEnd();
  } glPopMatrix();

  glutSwapBuffers();
}

void KeyboardFunc(unsigned char key, int x, int y) {
  if (key == 'n' || key == 'N')
    g_stalled = false;
  else if (key == 'q' || key == 'Q')
    exit(0);
}

void IdleFunc() {
  if (g_stalled == false) {
    g_main_engine->ProcessOneFrame();
    g_main_engine->GetImageEngine()->CurrentRGBDFrame(&g_rgb_image, nullptr);
    g_depth_image = g_main_engine->view()->depth_map;
    ConvertDepthToGreyscale(*g_depth_image, g_greyscale_image);
    g_stalled = true;
    glutPostRedisplay();
  }
}

int main(int argc, char* argv[]) {
  const char* calib_file = argv[1];
  const char* rgb_file = argv[2];
  const char* grey_file = argv[3];
  g_main_engine = new MainEngine(calib_file, rgb_file, grey_file);
  g_win_width = g_main_engine->view_size().x * 2;
  g_win_height = g_main_engine->view_size().y;
  g_greyscale_image = new ImageMono8u(g_main_engine->view_size().x * g_main_engine->view_size().y, MEM_CPU);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(g_win_width, g_win_height);
  glutCreateWindow("Test Window");

  glEnable(GL_TEXTURE_2D);
  glGenTextures(texture_n, textures);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, g_win_width, 0, g_win_height, 0, 1);

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutIdleFunc(IdleFunc);
  try {
    glutMainLoop();
  } catch (runtime_error e) {
    cout << e.what() << endl;
  }
  delete g_main_engine;
  delete g_greyscale_image;
  return 0;
}
