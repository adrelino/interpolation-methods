/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_GLHEADERS_H
#define INTERPOL_GLHEADERS_H

//#define GLEW_STATIC
//#include <GL/glew.h>

#define GL_GLEXT_PROTOTYPES
//#define GLFW_INCLUDE_GLCOREARB
//#define GLFW_INCLUDE_ES2
#define GLFW_INCLUDE_GLU   //#include <GL/glu.h>
#define GLFW_INCLUDE_GLEXT //#include <GL/glext.h>
#include <GLFW/glfw3.h>

#ifdef __EMSCRIPTEN__
/*
warning: undefined symbol: glColor3dv
warning: undefined symbol: glColor4dv
warning: undefined symbol: glLineStipple
warning: undefined symbol: glNormal3d
warning: undefined symbol: glPointSize
warning: undefined symbol: glPopAttrib
warning: undefined symbol: glPushAttrib
warning: undefined symbol: glVertex2d
warning: undefined symbol: glVertex3d
warning: undefined symbol: glVertex3dv
*/
#define glColor3dv(v)(glColor3f(v[0],v[1],v[2]))
#define glColor4dv(v)(glColor4f(v[0],v[1],v[2],v[3]))
#define glLineStipple(a,b)
#define glNormal3d glNormal3f
#define glPointSize(size)
#define glPopAttrib(a)
#define glPushAttrib(a)
#define glLightfv(light, pname, params)
#define glLightf(light, pname, param)
#define glVertex2d glVertex2f
#define glVertex3d glVertex3f
#define glVertex3dv(v)(glVertex3f(v[0],v[1],v[2]))
#endif


#define _USE_MATH_DEFINES
/*#include <GL/gl.h>
#include <GL/glu.h>*/
#include <vector>
#include <cmath>

#endif // INTERPOL_GLHEADERS_H
