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

#include <stdexcept>

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
    if (points[i].w < 0) {
      (*rgba)[i].set(0, 0, 0, 0);
    } else {
      const Vector4f& point = points[i];
      float depth = Tg(2, 0) * point.x + Tg(2, 1) * point.y + Tg(2, 2) * point.z;
      unsigned char greyscale = (depth > 2 || depth <= gMathFloatEpsilon) ? 0 : 255 - depth / 2 * 255;
      (*rgba)[i].set(greyscale, greyscale, greyscale, greyscale);
    }
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_RGB, GL_UNSIGNED_BYTE, g_rgb_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex2f(0, g_win_height / 3 * 2);                /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width / 3, g_win_height / 3 * 2);  /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width / 3, g_win_height / 3);      /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(0, g_win_height / 3);                    /* bottom-left */
    } glEnd();
    
    /* Display depth frame */
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g_greyscale_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex2f(0, g_win_height / 3);                /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width / 3, g_win_height / 3);  /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width / 3, 0);                 /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(0, 0);                               /* bottom-left */
    } glEnd();
    
    /* Display point cloud */
    glBindTexture(GL_TEXTURE_2D, textures[2]);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, g_main_engine->view_size().x, g_main_engine->view_size().y,
    //             0, GL_RGBA, GL_UNSIGNED_BYTE, g_pcl_image->GetData());
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                 g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g_main_engine->GetReconstructionEngine()->tsdf_map->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex2f(g_win_width / 3, g_win_height / 3 * 2);  /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width, g_win_height / 3 * 2);      /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width, 0);                         /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(g_win_width / 3, 0);                     /* bottom-left */
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
        
        glBindTexture(GL_TEXTURE_2D, textures[3 + i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, view_size_x, view_size_y,
                     0, GL_LUMINANCE, GL_UNSIGNED_BYTE, level.GetData());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBegin(GL_QUADS); {
          glTexCoord2f(0, 0); glVertex2f(anchor, 0);                                    /* top-left */
          glTexCoord2f(1, 0); glVertex2f(anchor + g_win_width / 3, 0);                  /* top-right */
          glTexCoord2f(1, 1); glVertex2f(anchor + g_win_width / 3, -g_win_height / 3);  /* bottom-right */
          glTexCoord2f(0, 1); glVertex2f(anchor, -g_win_height / 3);                    /* bottom-left */
        } glEnd();
        anchor += g_win_width / 3;
      }
    }
    
    /* Display ray length range */
  //glBindTexture(GL_TEXTURE_2D, textures[6]);
  //glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
  //             g_main_engine->view_size().x, g_main_engine->view_size().y,
  //             0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g_main_engine->GetReconstructionEngine()->tsdf_map->GetData());
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //glBegin(GL_QUADS); {
  //  glTexCoord2f(0, 0); glVertex2f(g_win_width, g_win_height / 3 * 2);                    /* top-left */
  //  glTexCoord2f(1, 0); glVertex2f(g_win_width + g_win_width / 3, g_win_height / 3 * 2);  /* top-right */
  //  glTexCoord2f(1, 1); glVertex2f(g_win_width + g_win_width / 3, g_win_height / 3);      /* bottom-right */
  //  glTexCoord2f(0, 1); glVertex2f(g_win_width, g_win_height / 3);                        /* bottom-left */
  //} glEnd();
    glBindTexture(GL_TEXTURE_2D, textures[7]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                 g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g_main_engine->GetRenderingEngine()->tsdf_map->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
    //glTexCoord2f(0, 0); glVertex2f(g_win_width, g_win_height / 3);                    /* top-left */
    //glTexCoord2f(1, 0); glVertex2f(g_win_width + g_win_width / 3, g_win_height / 3);  /* top-right */
    //glTexCoord2f(1, 1); glVertex2f(g_win_width + g_win_width / 3, 0);                 /* bottom-right */
    //glTexCoord2f(0, 1); glVertex2f(g_win_width, 0);                                   /* bottom-left */
      glTexCoord2f(0, 0); glVertex2f(g_win_width, g_win_height / 3 * 2);                        /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width + g_win_width / 3 * 2, g_win_height / 3 * 2);  /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width + g_win_width / 3 * 2, 0);                     /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(g_win_width, 0);                                           /* bottom-left */
    } glEnd();
    
    /* Display point cloud */
    glBindTexture(GL_TEXTURE_2D, textures[8]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, g_main_engine->view_size().x, g_main_engine->view_size().y,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, g_pcl_image->GetData());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBegin(GL_QUADS); {
      glTexCoord2f(0, 0); glVertex2f(g_win_width, 0);                                    /* top-left */
      glTexCoord2f(1, 0); glVertex2f(g_win_width + g_win_width / 3, 0);                  /* top-right */
      glTexCoord2f(1, 1); glVertex2f(g_win_width + g_win_width / 3, -g_win_height / 3);  /* bottom-right */
      glTexCoord2f(0, 1); glVertex2f(g_win_width, -g_win_height / 3);                    /* bottom-left */
    } glEnd();
  } glPopMatrix();

  glutSwapBuffers();
}

void KeyboardFunc(unsigned char key, int x, int y) {
  if (key == 'n' || key == 'N' || key == 13) {
    g_stalled = false;
    g_auto = false;
  } else if (key == 'a' || key == 'A') {
    g_auto ^= 1;
  } else if (key == 'q' || key == 'Q' || key == 27) {
    exit(0);
  }
}

void IdleFunc() {
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
    glutPostRedisplay();
  }
}

int main(int argc, char* argv[]) {
  const char* calib_file = argv[1];
  const char* rgb_file = argv[2];
  const char* grey_file = argv[3];
  g_main_engine = new MainEngine(calib_file, rgb_file, grey_file);
  g_win_width = g_main_engine->view_size().x * 1.8;
  g_win_height = g_main_engine->view_size().y * 1.8;
  g_greyscale_image = new ImageMono8u(g_main_engine->view_size().x * g_main_engine->view_size().y, MEM_CPU);
  g_pcl_image = new ImageRGBA8u(g_main_engine->view_size().x * g_main_engine->view_size().y, MEM_CPU);
  g_ray_length_min = new ImageMono8u(g_main_engine->GetRenderingEngine()->range_resolution()->x *
                                     g_main_engine->GetRenderingEngine()->range_resolution()->y, MEM_CPU);
  g_ray_length_max = new ImageMono8u(g_main_engine->GetRenderingEngine()->range_resolution()->x *
                                     g_main_engine->GetRenderingEngine()->range_resolution()->y, MEM_CPU);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

  glutInitWindowSize(g_win_width + g_win_width / 3 * 2, g_win_height);
  glutCreateWindow("Test Window");
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, g_win_width + g_win_width / 3 * 2, -g_win_height / 3, g_win_height / 3 * 2, 0, 10);

  glEnable(GL_TEXTURE_2D);
  glGenTextures(texture_n, textures);

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
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
