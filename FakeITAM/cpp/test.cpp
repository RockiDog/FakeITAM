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
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <stdexcept>
#include <vector>

#include "global_config.hpp"
#include "engines/depth_tracking_engine.hpp"
#include "engines/image_engine.hpp"
#include "engines/log_engine.hpp"
#include "engines/main_engine.hpp"
#include "engines/reconstruction_engine.hpp"
#include "engines/rendering_engine.hpp"
#include "engines/library/calibrator.hpp"
#include "engines/library/camera_pose.hpp"
#include "engines/library/point_cloud.hpp"
#include "engines/library/view_manager.hpp"
#include "engines/library/view_pyramid.hpp"

using namespace std;
using namespace fakeitam::config;
using namespace fakeitam::engine;
using namespace fakeitam::utility;

namespace {

MainEngine* g_main_engine = nullptr;
const ImageRGB8u* g_rgb_image = nullptr;
const ImageMono32f* g_depth_image = nullptr;
const ViewPyramid* g_view_pyramid = nullptr;
const GLsizei vbo_n = 4;
GLuint pcl_vbos[vbo_n];
GLuint color_vbos[vbo_n];

ImageMono8u* g_greyscale_image = nullptr;
ImageRGBA8u* g_pcl_image = nullptr;
ImageMono8u* g_ray_length_min = nullptr;
ImageMono8u* g_ray_length_max = nullptr;
bool g_stalled = false;
bool g_auto = false;
float g_win_width = 0;
float g_win_height = 0;

const GLsizei texture_n = 9;
GLuint textures[texture_n];

bool g_auto_rotate = false;
float g_rotate_angle_v = 0;
float g_rotate_angle_h = 0;
float g_translate_x = 0;
float g_translate_y = 0;
float g_translate_z = 0;
float g_scale = 1;

bool g_left_pressed = false, g_left_released = true;
bool g_right_pressed = false, g_right_released = true;
bool g_middle_pressed = false, g_middle_released = true;

int g_mouse_x, g_mouse_y;
int g_display_fun_id = 0;

vector<Vector3f> g_camera_poses;

}

void ConvertDepthsToGreyscales(const ImageMono32f& depths, ImageMono8u* greyscales) {
  for (int i = 0; i < depths.element_n(); ++i) {
    float depth = depths[i];
    unsigned char greyscale = (depth > 1 || depth <= gMathFloatEpsilon) ? 0 : 255 - depth * 255;
    (*greyscales)[i] = greyscale;
  }
}

void ConvertRayLengthRangesToGreyscales(const MemBlock<Vector2f>& ray_length_ranges,
                                              ImageMono8u* min, ImageMono8u* max) {
  for (int i = 0; i < ray_length_ranges.element_n(); ++i) {
    float depth_min = ray_length_ranges[i].x;
    float depth_max = ray_length_ranges[i].y;
    unsigned char greyscale_min = (depth_min > 1 || depth_min <= gMathFloatEpsilon) ? 0 : 255 - depth_min * 255;
    unsigned char greyscale_max = (depth_max > 1 || depth_max <= gMathFloatEpsilon) ? 0 : 255 - depth_max * 255;
    (*min)[i] = greyscale_min;
    (*max)[i] = greyscale_max;
  }
}

void ProjectGlobal3DCoordinatesToCurrentPerspectiveThenConvertTheirFuckingDepthsToRGBAs(
    const MemBlock<Vector4f>& points,
    const Matrix4f& Tg,
    ImageRGBA8u* rgba) {
  for (int i = 0; i < points.element_n(); ++i) {
    if (points[i].w <= 0) {
      (*rgba)[i].set(0, 0, 0, 0);
    } else {
      const Vector4f& point = points[i];
      float depth = Tg(2, 0) * point.x + Tg(2, 1) * point.y + Tg(2, 2) * point.z;
      unsigned char greyscale = (depth > 2 || depth <= gMathFloatEpsilon) ? 0 : 255 - depth / 2 * 255;
      (*rgba)[i].set(greyscale, greyscale, greyscale, greyscale);
    }
  }
}

void DisplayFunc0() {
  if (g_rgb_image == nullptr)
    return;

  /* Do the actual drawing */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor4f(1, 1, 1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glPushMatrix(); {
    /* Display RGB image */
    glBindTexture(GL_TEXTURE_2D, textures[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_RGB, GL_UNSIGNED_BYTE, g_rgb_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex3f(0, g_win_height / 3 * 2, -100);                /* top-left */
      glTexCoord2f(1, 0); glVertex3f(g_win_width / 3, g_win_height / 3 * 2, -100);  /* top-right */
      glTexCoord2f(1, 1); glVertex3f(g_win_width / 3, g_win_height / 3, -100);      /* bottom-right */
      glTexCoord2f(0, 1); glVertex3f(0, g_win_height / 3, -100);                    /* bottom-left */
    } glEnd();
    
    /* Display depth frame */
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g_greyscale_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex3f(0, g_win_height / 3, -100);                /* top-left */
      glTexCoord2f(1, 0); glVertex3f(g_win_width / 3, g_win_height / 3, -100);  /* top-right */
      glTexCoord2f(1, 1); glVertex3f(g_win_width / 3, 0, -100);                 /* bottom-right */
      glTexCoord2f(0, 1); glVertex3f(0, 0, -100);                               /* bottom-left */
    } glEnd();
    
    /* Display the point cloud */
    glBindTexture(GL_TEXTURE_2D, textures[2]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, g_pcl_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex3f(g_win_width / 3, g_win_height / 3 * 2, -100);  /* top-left */
      glTexCoord2f(1, 0); glVertex3f(g_win_width, g_win_height / 3 * 2, -100);      /* top-right */
      glTexCoord2f(1, 1); glVertex3f(g_win_width, 0, -100);                         /* bottom-right */
      glTexCoord2f(0, 1); glVertex3f(g_win_width / 3, 0, -100);                     /* bottom-left */
    } glEnd();
    
    /* Display view pyramid */
    if (g_view_pyramid != nullptr) {
      float anchor = 0;
      float view_size_x, view_size_y;
      for (int i = 0; i < g_view_pyramid->level_n(); ++i) {
        view_size_x = g_view_pyramid->LevelAt(i)->size.x;
        view_size_y = g_view_pyramid->LevelAt(i)->size.y;
        ImageMono8u level(view_size_x * view_size_y, MEM_CPU);
        ConvertDepthsToGreyscales(*g_view_pyramid->LevelAt(i)->depth_map, &level);
        
        glBindTexture(GL_TEXTURE_2D, textures[4 + i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, view_size_x, view_size_y,
                     0, GL_LUMINANCE, GL_UNSIGNED_BYTE, level.GetData());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBegin(GL_QUADS); {
          glTexCoord2f(0, 0); glVertex3f(anchor, 0, -100);                                    /* top-left */
          glTexCoord2f(1, 0); glVertex3f(anchor + g_win_width / 3, 0, -100);                  /* top-right */
          glTexCoord2f(1, 1); glVertex3f(anchor + g_win_width / 3, -g_win_height / 3, -100);  /* bottom-right */
          glTexCoord2f(0, 1); glVertex3f(anchor, -g_win_height / 3, -100);                    /* bottom-left */
        } glEnd();
        anchor += g_win_width / 3;
      }
    }
  } glPopMatrix();
  glDisable(GL_TEXTURE_2D);

  glutSwapBuffers();
}

void DisplayFunc1() {
  if (g_rgb_image == nullptr)
    return;

  /* Do the actual drawing */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor4f(1, 1, 1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glPushMatrix(); {
    glRotatef(180, 1, 0, 0);
    glTranslatef(g_translate_x + 800, g_translate_y, g_translate_z + 500);
    glRotatef(g_rotate_angle_h, 0, 1, 0);
    glRotatef(g_rotate_angle_v, 1, 0, 0);
    
    glColor3f(1, 0, 0);
    glPushMatrix(); {
      glTranslatef(g_camera_poses.front().x / gVoxelMetricSize,
                   g_camera_poses.front().y / gVoxelMetricSize,
                   g_camera_poses.front().z / gVoxelMetricSize);
      glutWireSphere(g_scale, 10, 10);
    } glPopMatrix();
    glColor3f(0.5, 0.5, 0.5);
    for (auto it = g_camera_poses.cbegin() + 1; it != g_camera_poses.cend(); ++it) {
      glPushMatrix(); {
        glTranslatef(it->x / gVoxelMetricSize,
                     it->y / gVoxelMetricSize,
                     it->z / gVoxelMetricSize);
        glutWireSphere(g_scale, 10, 10);
      } glPopMatrix();
    }
    
    glScalef(g_scale / gVoxelMetricSize, g_scale / gVoxelMetricSize, g_scale / gVoxelMetricSize);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    
    //  glEnableClientState(GL_COLOR_ARRAY);
    //  glBindBuffer(GL_ARRAY_BUFFER, pcl_vbos[0]);
    //  glVertexPointer(3, GL_FLOAT, 0, nullptr);
    //  glBufferData(GL_ARRAY_BUFFER,
    //               sizeof(float) * 3 * g_main_engine->GetReconstructionEngine()->pcl_cnt,
    //               g_main_engine->GetReconstructionEngine()->pcl->GetData(),
    //               GL_STATIC_DRAW);
    //  glBindBuffer(GL_ARRAY_BUFFER, color_vbos[0]);
    //  glColorPointer(3, GL_FLOAT, 0, nullptr);
    //  glBufferData(GL_ARRAY_BUFFER,
    //               sizeof(float) * 3 * g_main_engine->GetReconstructionEngine()->pcl_cnt2,
    //               g_main_engine->GetReconstructionEngine()->pcl2->GetData(),
    //               GL_STATIC_DRAW);
    //  glDrawArrays(GL_POINTS, 0, g_main_engine->GetReconstructionEngine()->pcl_cnt2);
    //  glDisableClientState(GL_COLOR_ARRAY);
    //  
    //  glColor3f(0, 0.8, 0);
    //  glBindBuffer(GL_ARRAY_BUFFER, pcl_vbos[1]);
    //  glVertexPointer(3, GL_FLOAT, 0, nullptr);
    //  glBufferData(GL_ARRAY_BUFFER,
    //               sizeof(float) * 3 * g_main_engine->GetRenderingEngine()->pcl_cnt,
    //               g_main_engine->GetRenderingEngine()->pcl->GetData(),
    //               GL_STATIC_DRAW);
    //  glDrawArrays(GL_POINTS, 0, g_main_engine->GetReconstructionEngine()->pcl_cnt);
    //  
    //  glColor3f(0, 0, 0.8);
    //  glBindBuffer(GL_ARRAY_BUFFER, pcl_vbos[2]);
    //  glVertexPointer(3, GL_FLOAT, 0, nullptr);
    //  glBufferData(GL_ARRAY_BUFFER,
    //               sizeof(float) * 3 * g_main_engine->GetRenderingEngine()->pcl_cnt2,
    //               g_main_engine->GetRenderingEngine()->pcl2->GetData(),
    //               GL_STATIC_DRAW);
    //  glDrawArrays(GL_POINTS, 0, g_main_engine->GetRenderingEngine()->pcl_cnt2);
    
    glColor3f(1, 1, 1);
    glPointSize(2);
    glBindBuffer(GL_ARRAY_BUFFER, pcl_vbos[3]);
    glVertexPointer(3, GL_FLOAT, sizeof(float) * 4, nullptr);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(float) * 4 * g_main_engine->point_cloud()->locations()->element_n(),
                 g_main_engine->point_cloud()->locations()->GetData(),
                 GL_STATIC_DRAW);
    glDrawArrays(GL_POINTS, 0, g_main_engine->point_cloud()->locations()->element_n());
    glDisableClientState(GL_VERTEX_ARRAY);
  }; glPopMatrix();

  glPushMatrix(); {
    glColor3f(0.1, 0.1, 0.1);
    glTranslatef(10 + g_scale / 2,
                 -g_win_height / 3 + 10 + g_scale / 2,
                 -(10 + g_scale / 2));
    glScalef(g_scale, g_scale, g_scale);
    glutSolidCube(1);
  } glPopMatrix();

  glutSwapBuffers();
}

void DisplayFunc() {
  switch (g_display_fun_id) {
    case 0 : DisplayFunc0(); break;
    case 1 : DisplayFunc1(); break;
  }
}

void KeyboardFunc(unsigned char key, int x, int y) {
  switch (key) {
    case 'f' :
    case 'F' : { g_display_fun_id = 1 - g_display_fun_id; DisplayFunc(); } break;
    
    case 'n' :
    case 'N' :
    case 13  : { g_stalled = false; g_auto = false; } break;
    case 'a' :
    case 'A' : { g_auto ^= 1; } break;
    case 'q' :
    case 'Q' :
    case 27  : { exit(0); } break;
    
    case 'r' :
    case 'R' : { g_auto_rotate ^= 1; } break;
    case 'o' :
    case 'O' : {
      g_rotate_angle_h = g_rotate_angle_v = 0;
      g_translate_x = g_translate_y = g_translate_z = 0;
      g_scale = 1;
    } break;
  }
}

void SpecialKeyFunc(int key, int x, int y) {
  switch (key) {
    case GLUT_KEY_LEFT  : { if (g_right_released) g_translate_x -= 10; } break;
    case GLUT_KEY_RIGHT : { if (g_right_released) g_translate_x += 10; } break;
    case GLUT_KEY_UP    : { if (g_right_released) g_translate_y -= 10; } break;
    case GLUT_KEY_DOWN  : { if (g_right_released) g_translate_y += 10; } break;
  }
}

void MouseFunc(int button,int state,int x,int y) {
  switch (button) {
    case GLUT_LEFT_BUTTON : {
      if (state == GLUT_DOWN) {
        if (g_left_released) {
          g_mouse_x = x;
          g_mouse_y = y;
        }
        g_left_pressed = true;
        g_left_released = false;
      } else if (state == GLUT_UP) {
        g_left_pressed = false;
        g_left_released = true;;
      }
    } break;
    case GLUT_RIGHT_BUTTON : {
      if (state == GLUT_DOWN) {
        if (g_right_released) {
          g_mouse_x = x;
          g_mouse_y = y;
        }
        g_right_pressed = true;
        g_right_released = false;
      } else if (state == GLUT_UP) {
        g_right_pressed = false;
        g_right_released = true;
      }
    } break;
    case GLUT_MIDDLE_BUTTON : {
      if (state == GLUT_DOWN) {
        if (g_middle_released) {
          g_mouse_x = x;
          g_mouse_y = y;
        }
        g_middle_pressed = true;
        g_middle_released = false;
      } else if (state == GLUT_UP) {
        g_middle_pressed = false;
        g_middle_released = true;
      }
    } break;
  }
}

void MouseMotionFunc(int x, int y) {
  if (g_left_pressed) {
    g_rotate_angle_h += (x - g_mouse_x) / 10.0;
    g_rotate_angle_v -= (y - g_mouse_y) / 10.0;
    while (g_rotate_angle_h >= 360) g_rotate_angle_h -= 360;
    while (g_rotate_angle_v >= 360) g_rotate_angle_v -= 360;
    while (g_rotate_angle_h < 0) g_rotate_angle_h += 360;
    while (g_rotate_angle_v < 0) g_rotate_angle_v += 360;
    g_mouse_x = x;
    g_mouse_y = y;
  }

  if (g_right_pressed) {
    g_translate_x += x - g_mouse_x;
    g_translate_y += y - g_mouse_y;
    g_mouse_x = x;
    g_mouse_y = y;
  }

  if (g_middle_pressed) {
    g_scale += (x - g_mouse_x) / 100.0;
    if (g_scale < 0.5) g_scale = 0.5;
    //if (g_scale > 5.0) g_scale = 5.0;
    g_mouse_x = x;
    g_mouse_y = y;
  }
}

void IdleFunc() {
  if (g_auto_rotate && g_left_released)
    g_rotate_angle_h = g_rotate_angle_h == 0 ? 359 : g_rotate_angle_h - 1;

  if (g_stalled == false || g_auto == true) {
    g_main_engine->ProcessOneFrame();
    g_main_engine->GetImageEngine()->CurrentRGBDFrame(&g_rgb_image, nullptr);
    g_depth_image = g_main_engine->view()->depth_map;
    g_view_pyramid = dynamic_cast<const DepthTrackingEngine*>(g_main_engine->GetTrackingEngine())->view_pyramid();
    ConvertDepthsToGreyscales(*g_depth_image, g_greyscale_image);
    ConvertRayLengthRangesToGreyscales(*g_main_engine->GetRenderingEngine()->ray_length_range(),
                                       g_ray_length_min, g_ray_length_max);
    ProjectGlobal3DCoordinatesToCurrentPerspectiveThenConvertTheirFuckingDepthsToRGBAs(
        *g_main_engine->point_cloud()->locations(),
        g_main_engine->camera_pose()->m,
        g_pcl_image);
    g_stalled = true;
    g_camera_poses.push_back(g_main_engine->camera_pose()->t());
  }

  glutPostRedisplay();
}

int main(int argc, char* argv[]) {
  const char* calib_file = argv[1];
  const char* rgb_file = argv[2];
  const char* grey_file = argv[3];
  g_main_engine = new MainEngine(calib_file, rgb_file, grey_file);
  g_win_width = g_main_engine->view_size().x * 1.7;
  g_win_height = g_main_engine->view_size().y * 1.7;
  g_greyscale_image = new ImageMono8u(g_main_engine->view_size().x * g_main_engine->view_size().y, MEM_CPU);
  g_pcl_image = new ImageRGBA8u(g_main_engine->view_size().x * g_main_engine->view_size().y, MEM_CPU);
  g_ray_length_min = new ImageMono8u(g_main_engine->GetRenderingEngine()->range_resolution()->x *
                                     g_main_engine->GetRenderingEngine()->range_resolution()->y, MEM_CPU);
  g_ray_length_max = new ImageMono8u(g_main_engine->GetRenderingEngine()->range_resolution()->x *
                                     g_main_engine->GetRenderingEngine()->range_resolution()->y, MEM_CPU);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);

  glutInitWindowSize(g_win_width + g_win_width / 3 * 2, g_win_height);
  glutCreateWindow("Test Window");

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, g_win_width + g_win_width / 3 * 2, -g_win_height / 3, g_win_height / 3 * 2, 0, 10000);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
  glGenTextures(texture_n, textures);
  glGenBuffers(vbo_n, pcl_vbos);
  glGenBuffers(vbo_n, color_vbos);

  glutKeyboardFunc(KeyboardFunc);
  glutSpecialFunc(SpecialKeyFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MouseMotionFunc);
  glutDisplayFunc(DisplayFunc);
  glutIdleFunc(IdleFunc);
  try {
    glutMainLoop();
  } catch (runtime_error e) {
    LOG->Write("\n\n")->WriteLineF(E, "%s\n", e.what());
    throw e;
  }
  delete g_main_engine;
  delete g_greyscale_image;
  delete g_pcl_image;
  return 0;
}
