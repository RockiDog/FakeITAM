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

using namespace std;
using namespace fakeitam::engine;

namespace {

MainEngine* g_main_engine = nullptr;
const ImageRGB8u*g_rgb_image = nullptr;
bool g_stalled = false;
int g_win_width = 0;
int g_win_height = 0;

}

void DisplayFunc() {
  if (g_rgb_image == nullptr)
    return;

  /* Do the actual drawing */
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1.0f, 1.0f, 1.0f);
  glEnable(GL_TEXTURE_2D);

  GLsizei texture_n = 1;
  GLuint texture;
  glGenTextures(texture_n, &texture);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
      glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_win_width, g_win_height,
                     0, GL_RGB, GL_UNSIGNED_BYTE, g_rgb_image->GetData());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBegin(GL_QUADS);
          glTexCoord2f(0, 0);
          glVertex2f(0, g_win_height);            /* top-left */
          
          glTexCoord2f(g_win_width, 0);
          glVertex2f(g_win_width, g_win_height);  /* top-right */
          
          glTexCoord2f(g_win_width, g_win_height);
          glVertex2f(g_win_width, 0);             /* bottom-right */
          
          glTexCoord2f(0, g_win_height);
          glVertex2f(0, 0);                       /* bottom-right */
        glEnd();
      glDisable(GL_TEXTURE_2D);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glColor3f(1.0f, 0.0f, 0.0f);
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
    g_stalled = true;
    glutPostRedisplay();
  }
}

int main(int argc, char* argv[]) {
  const char* calib_file = argv[1];
  const char* rgb_file = argv[2];
  const char* grey_file = argv[3];
  g_main_engine = new MainEngine(calib_file, rgb_file, grey_file);
  g_win_width = g_main_engine->view_size().x;
  g_win_height = g_main_engine->view_size().y;

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(g_win_width, g_win_height);
  glutCreateWindow("Test Window");
  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutIdleFunc(IdleFunc);
  try {
    glutMainLoop();
  } catch (runtime_error e) {
    cout << e.what() << endl;
  }
  delete g_main_engine;
  return 0;
}
