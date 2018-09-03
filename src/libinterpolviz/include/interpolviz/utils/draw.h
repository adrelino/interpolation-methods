/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_DRAW_H
#define INTERPOL_DRAW_H

#include <Eigen/Dense>
#include "glheaders.h"
#include "frustum.h"
#include "interpolviz/trajectory/Pose3.h"

namespace interpol {

#define RED 0
#define GREEN 1
#define BLUE 2
#define ORANGE 3
#define MAG 4
#define CYAN 5
#define BLACK 6
#define GRAY 7

#define ALPHA 0.3
#define MAX_2D 10.0

namespace Colormap {

static Eigen::Vector4d RED1    (0.8, 0.0, 0.0, ALPHA);
//static Vector4d RED2    (0.4, 0.0, 0.0, 0.5);
static Eigen::Vector4d GREEN1  (0.0, 0.8, 0.0, ALPHA);
//static Vector4d GREEN2  (0.0, 0.4, 0.0, 0.5);
static Eigen::Vector4d BLUE1   (0.0, 0.0, 0.8, ALPHA);
//static Vector4d BLUE2   (0.0, 0.0, 0.4, 0.5);
static Eigen::Vector4d ORANGE1 (1.0, 0.6, 0.0, ALPHA);
//static Vector4d ORANGE2 (0.5, 0.3, 0.0, 0.5);
static Eigen::Vector4d MAG1    (0.3, 0.0, 0.8, ALPHA);
//static Vector4d MAG2    (0.1, 0.0, 0.5, 0.5);
static Eigen::Vector4d CYAN1   (0.0, 0.6, 1.0, ALPHA);
//static Vector4d CYAN2   (0.0, 0.1, 0.5, 0.5);
static Eigen::Vector4d BLACK1  (0.0, 0.0, 0.0, ALPHA);
//static Vector4d BLACK2  (0.2, 0.2, 0.2, 0.5);
static Eigen::Vector4d GRAY1  (0.35,0.35,0.35,0.8); //as small spheres //glColor4d(0.35,0.35,0.35,0.8); //gray


static Eigen::Vector4d colors1[8] = {RED1,GREEN1,BLUE1,ORANGE1,MAG1,CYAN1,BLACK1,GRAY1};
static Eigen::Vector4d get(int colorChoice){return colors1[colorChoice];}

}

namespace visualize {
class SolidSphere
{
protected:
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> normals;
    //std::vector<GLfloat> texcoords;
    std::vector<GLushort> indices;

public:
    SolidSphere(float radius, unsigned int rings, unsigned int sectors)
    {
        float const R = 1./(float)(rings-1);
        float const S = 1./(float)(sectors-1);
        int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
//        texcoords.resize(rings * sectors * 2);
        std::vector<GLfloat>::iterator v = vertices.begin();
        std::vector<GLfloat>::iterator n = normals.begin();
        //std::vector<GLfloat>::iterator t = texcoords.begin();
        for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
                float const y = sin( -M_PI_2 + M_PI * r * R );
                float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
                float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

//                *t++ = s*S;
//                *t++ = r*R;

                *v++ = x * radius;
                *v++ = y * radius;
                *v++ = z * radius;

                *n++ = x;
                *n++ = y;
                *n++ = z;
            }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
        for(r = 0; r < rings-1; r++) for(s = 0; s < sectors-1; s++) {
                *i++ = r * sectors + s;
                *i++ = r * sectors + (s+1);
                *i++ = (r+1) * sectors + (s+1);
                *i++ = (r+1) * sectors + s;
            }
    }

    void draw(){
        //glColor4d(0.0,0.0,0.8,0.3);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        //glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);
        //glTexCoordPointer(2, GL_FLOAT, 0, &texcoords[0]);
        glDrawElements(GL_QUADS, indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    void draw(const Eigen::Vector3d& tra, const Eigen::Matrix3d& rot, const Eigen::Vector3d& scale)
    {
        glPushMatrix();

        Eigen::Affine3d T;
        T.linear() = rot;
        T.translation() = tra;
        T.scale(scale);

/*        Matrix4d m = Matrix4d::Zero();
        m.topLeftCorner<3,3>() = rot;
        m(3,3)=1;*/

        glMultMatrixd(T.data());

        draw();

/*        glTranslated(tra.x(),tra.y(),tra.z());
        glScaled(scale.x(),scale.y(),scale.z());*/

        glPopMatrix();


    }
};

static SolidSphere nanosphere(0.025, 6, 12);
static SolidSphere tinysphere(0.05, 6, 12);
static SolidSphere sphere(1, 12, 24);
static SolidSphere sphere2(0.99, 24, 48);


static void drawFrustumIntrinsics(Eigen::Vector4d& colorLine, Eigen::Vector4d& colorPlane){
    double f_x =524;// K(0,0),
    double f_y = 540; //K(1,1),
    double c_x = 320; //K(1,2);
    double c_y = 240; //K(1,2);

    // Assuming that this is an ideal camera (c_y and c_x are at the center of the image)
    double fovy = 2.0 * atan2(c_y, f_y) * 180 / M_PI;
    double aspect_ratio = c_x / c_y;

    glPushMatrix();
    glScaled(4,4,4);
    drawFrustum(fovy,aspect_ratio,-0.02f,colorLine,colorPlane);
    glPopMatrix();
}


static void drawCubeImmediateMode(double sizeDouble){
    float size = sizeDouble/2.0;
    
/*    glBegin(GL_LINE_STRIP);
    glVertex3d(s,s,s);
    glVertex3d(s,s,-s);
    glVertex3d(s,-s,-s);
    glVertex3d(s,-s,s);
    glVertex3d(s,s,s);

    s*=-1;
    glVertex3d(s,s,s);
    glVertex3d(s,s,-s);
    glVertex3d(s,-s,-s);
    glVertex3d(s,-s,s);
    glVertex3d(s,s,s);
    glEnd();*/


    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = size)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    //glColor3f(0.0f, size, 0.0f);     // Green
    glVertex3f( size, size, -size);
    glVertex3f(-size, size, -size);
    glVertex3f(-size, size,  size);
    glVertex3f( size, size,  size);

    // Bottom face (y = -size)
    //glColor3f(size, 0.5f, 0.0f);     // Orange
    glVertex3f( size, -size,  size);
    glVertex3f(-size, -size,  size);
    glVertex3f(-size, -size, -size);
    glVertex3f( size, -size, -size);

    // Front face  (z = size)
    //glColor3f(size, 0.0f, 0.0f);     // Red
    glVertex3f( size,  size, size);
    glVertex3f(-size,  size, size);
    glVertex3f(-size, -size, size);
    glVertex3f( size, -size, size);

    // Back face (z = -size)
    //glColor3f(size, size, 0.0f);     // Yellow
    glVertex3f( size, -size, -size);
    glVertex3f(-size, -size, -size);
    glVertex3f(-size,  size, -size);
    glVertex3f( size,  size, -size);

    // Left face (x = -size)
    //glColor3f(0.0f, 0.0f, size);     // Blue
    glVertex3f(-size,  size,  size);
    glVertex3f(-size,  size, -size);
    glVertex3f(-size, -size, -size);
    glVertex3f(-size, -size,  size);

    // Right face (x = size)
    //glColor3f(size, 0.0f, size);     // Magenta
    glVertex3f(size,  size, -size);
    glVertex3f(size,  size,  size);
    glVertex3f(size, -size,  size);
    glVertex3f(size, -size, -size);
    glEnd();  // End of drawing color-cube
}

static void drawCube(const float *vertices, const GLuint *indices){
        // select background color to be black
        float R = 0, G = 0, B = 0, alpha = 0;
        glClearColor(R, G, B, alpha);

        // clear all pixels in the window with the color selected above
        glClear(GL_COLOR_BUFFER_BIT);

        // enable use of vertex coordinate information from the array
        glEnableClientState(GL_VERTEX_ARRAY);

        glVertexPointer(3,                // number of coordinates per vertex (X,Y,Z)
                        GL_FLOAT,         // type of numbers
                        sizeof(float)*7,  // stride - gap between each set of (X,Y,Z)
                        &vertices[0]);    // offset - location of initial (X,Y,Z)

        // enable use of vertex color information from the array
        glEnableClientState(GL_COLOR_ARRAY);

        glColorPointer(4,                 // number of color values per vertex (R,G,B,A)
                       GL_FLOAT,          // type of values
                       sizeof(float)*7,   // stride - gap between each set of (R,G,B,A)
                       &vertices[3]);     // offset - location of initial (R,G,B,A)

        // enable use of indexing information
        glEnableClientState(GL_INDEX_ARRAY);
        glIndexPointer(GL_UNSIGNED_INT, 0, indices);

        // increase line width (the default value is 1)
        glLineWidth(3);

        // draw command
        glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, indices);
}

static void drawCube(){
// vertex data for drawing a cube (8 vertices)
   const float vertices[] =
   {
   //   X     Y     Z   R  G  B  A
     -0.5, -0.5, -0.5,  1, 0, 0, 1, // vertex 0
      0.5, -0.5, -0.5,  0, 1, 0, 1, // vertex 1
      0.5,  0.5, -0.5,  1, 0, 1, 1, // vertex 2
     -0.5,  0.5, -0.5,  1, 1, 0, 1, // vertex 3
     -0.5, -0.5,  0.5,  1, 0, 0, 1, // vertex 4
      0.5, -0.5,  0.5,  1, 0, 0, 1, // vertex 5
      0.5,  0.5,  0.5,  0, 1, 1, 1, // vertex 6
     -0.5,  0.5,  0.5,  1, 0, 1, 1  // vertex 7
   };

   // indices data for drawing the 12 edges
   const GLuint indices[] =
   {
       0, 1,  // edge 0
       1, 2,  // edge 1
       2, 3,  // edge 2
       3, 0,  // edge 3
       4, 5,  // edge 4
       5, 6,  // edge 5
       6, 7,  // edge 6
       7, 4,  // edge 7
       0, 4,  // edge 8
       1, 5,  // edge 9
       2, 6,  // edge 10
       3, 7   // edge 11
   };
    drawCube(vertices,indices);
}

static void draw2dLine(const std::vector<double> &vs, int color, double max = MAX_2D){
    glColor3dv(Colormap::colors1[color].data());
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    double n = vs.size();
    for (int i = 0; i < n; ++i) {
        double x = i*1.0/(n-1.0);
        double y = vs[i]/max;
        //cout<<n<<" : "<<x<<","<<y<<endl;
        glVertex2d(x,y);
    }
    glEnd();

    glColor4dv(Colormap::colors1[color].data());
    glLineWidth(1);
    glPushAttrib(GL_ENABLE_BIT);glLineStipple(2, 0xAAAA);glEnable(GL_LINE_STIPPLE);
    glBegin(GL_LINE_STRIP);
    glVertex2d(0,1/max);
    glVertex2d(1,1/max);
    glEnd();
    glPopAttrib();
}

static void draw2dPoints(const interpol::Poses3d& vs, int color, int test, double max = MAX_2D){
    glColor4dv(Colormap::colors1[color].data());
    glPointSize(4);
    glBegin(GL_POINTS);
    double n = vs.size();
    for (int i = 0; i < n; ++i) {
        double x = i*1.0/(n-1.0);
        //double y = vs[i].getDeriv(test).norm()/max;
        // cout<<n<<" : "<<x<<","<<y<<endl;
        //glVertex2d(x,y);
    }
    glEnd();
}

static void drawPointNorm(const Eigen::Vector3d& vec, int color, double x, double max = MAX_2D){
    glColor4dv(Colormap::colors1[color].data());
    glPointSize(6);
    glBegin(GL_POINTS);
    double y = vec.norm()/max;
    glVertex2d(x,y);
    glEnd();
}


static Eigen::Vector3d glVertex4dvQuaternion(const Eigen::Vector4d& coeffs, const int& removeDim){
        double x = coeffs.x();
        double y = coeffs.y();
        double z = coeffs.z();

        if(removeDim==1){ /* WYZ */
            x=coeffs.w();
        }else if (removeDim==2){ /* WXZ */
            x=coeffs.w();
            y=coeffs.x();
        }else if(removeDim==3){ /* WXY */
            x=coeffs.w();
            y=coeffs.x();
            z=coeffs.y();
        }else{
            /* WXY */
        }

        if(coeffs[removeDim]<0){

        }

        return Eigen::Vector3d(x,y,z);

        //glVertex3d(x,y,z);


/*    switch(removeDim){
        case 0 : glVertex3dv(coeffs.data()); break; //w-> x y z
        case 1 : glVertex3d(coeffs.w(),coeffs.y(),coeffs.z()); break;//x-> w y z
        case 2 : glVertex3d(coeffs.w(),coeffs.y(),coeffs.z()); break;// y-> w x z
        case 3 : glVertex3d(coeffs.w(),coeffs.y(),coeffs.z()); break;//z -> w x y
    }*/
    }


/**
 * draws cylinder towards z direction
 */
static void drawCylinder(double r, double l, bool coverback=true, bool coverfront=true, bool normalInwards=false){
    int i;
    int n = 20;

    int z=normalInwards ? -1 : 1;

    for(i=0;i<2*n;i++){
        glBegin(GL_POLYGON);
        // Explanation: the normal of the whole polygon is the coordinate of the center of the polygon for a sphere
        glNormal3d(sin((i+0.5)*M_PI/n)*z,cos((i+0.5)*M_PI/n)*z,0); //middle
        glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),0);
        glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),0);
        glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),l);
        glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),l);
        glEnd();
        //bottom
        if(coverback){
            glBegin(GL_POLYGON);
            glNormal3d(0,0,-1*z); //middle
            glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),0);
            glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),0);
            glVertex3d(0,0,0);
            glEnd();
        }
        //top
        if(coverfront){
            glBegin(GL_POLYGON);
            glNormal3d(0,0,1*z); //middle
            glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),l);
            glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),l);
            glVertex3d(0,0,l);
            glEnd();
        }
    }
}

static void drawOrigin(){
    glPushMatrix();
    glColor3d(0,0,1);
    drawCylinder(0.01,1); //z blue
    glRotatef(90,0,1,0);
    glColor3d(1,0,0);
    drawCylinder(0.01,1); //x  red
    glRotatef(-90,1,0,0);
    glColor3d(0,1,0);
    drawCylinder(0.01,1); //y  green
    glPopMatrix();
}

static void drawOriginScaled(double x, double y, double z){
    glPushMatrix();
    glColor3d(0,0,1);
    drawCylinder(z/20.0,z); //z blue
    glRotatef(90,0,1,0);
    glColor3d(1,0,0);
    drawCylinder(x/20.0,x); //x  red
    glRotatef(-90,1,0,0);
    glColor3d(0,1,0);
    drawCylinder(y/20.0,y); //y  green
    glPopMatrix();
}

    static void drawIso3d(const Eigen::Isometry3d& p, int colorChoice=0, double scale=3.0){
        glPushMatrix();
        glMultMatrixd(p.data());
        glScaled(scale,scale,scale);
        //drawOriginScaled(0.1,0.1,0.1);
        drawFrustumIntrinsics(Colormap::colors1[colorChoice],Colormap::colors1[colorChoice]);
        glPopMatrix();
    }

}

} // ns interpol

#endif // INTERPOL_DRAW_H
