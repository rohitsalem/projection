// Date: 26th September 2017
// Author: rsalem
// Purpose: Convert 3d world coordinates to image pixels
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <cmath>

#include "geometry.h"
float h = 2;
float b = 0.75;
float w = 1;

Vec3f pA_o(b/2,w/2,0); // Vertices of cuboid with respect to the object's origin.
Vec3f pB_o(-b/2,w/2,0);
Vec3f pC_o(-b/2,-w/2,0);
Vec3f pD_o(b/2,-w/2,0);
Vec3f pE_o(b/2,w/2,h);
Vec3f pF_o(-b/2,w/2,h);
Vec3f pG_o(-b/2,-w/2,h);
Vec3f pH_o(b/2,-w/2,h);

Vec3f pC_w(-1.490300, 0.025726, 2.22266);
Vec3f pO_w(4.706141, 1.250742, 0.148441);
Matrix44f RT;
void computeTransformationMatrix(float r , float p, float y, Vec3f c)
{
  Matrix44f Rx(1, 0, 0, 0, 0, cos(r), -sin(r), 0, 0, sin(r), cos(r), 0, 0, 0, 0, 1);
  Matrix44f Ry(cos(p), 0, sin(p), 0, 0, 1, 0, 0, -sin(p), 0, cos(p), 0, 0, 0, 0, 1);
  Matrix44f Rz(cos(y), -sin(p), 0, 0, sin(p), cos(p), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  Matrix44f T(1,0,0,0,0,1,0,0,0,0,1,0,c[0],c[1],c[2],1);

  Matrix44f Rxy;
  Matrix44f Rxyz;

  Matrix44f::multiply(Rx,Ry,Rxy);
  Matrix44f::multiply(Rxy, Rz, Rxyz);
  Matrix44f::multiply( Rxyz, T, RT);

}

void computePixelCoordinates(
    const Vec3f pWorld,
    Vec2i &pRaster,
    const Matrix44f &worldToCamera,
    const float &canvasWidth,
    const float &canvasHeight,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight
)
{
    Vec3f pCamera;
    worldToCamera.multVecMatrix(pWorld, pCamera);
    Vec2f pScreen;
    pScreen.x = pCamera.x / -pCamera.z;
    pScreen.y = pCamera.y / -pCamera.z;
    Vec2f pNDC;
    pNDC.x = (pScreen.x + canvasWidth * 0.5) / canvasWidth;
    pNDC.y = (pScreen.y + canvasHeight * 0.5) / canvasHeight;
    pRaster.x = (int)(pNDC.x * imageWidth);
    pRaster.y = (int)((1 - pNDC.y) * imageHeight);
}

int main (int agrc, char **argv)
{
  //4x4 matrix goes here:
  // Matrix44f cameraToWorld(#####################)
  // Matrix44f worldToCamera = cameraToWorld.inverse();

  // define canvas width & height
  float canvasWidth = 2, canvasHeight =2 ;
  // define image height and width here
  uint32_t imageWidth = 512, imageHeight = 512;
  float rC = 0 , pC = 0.15 , yC = 0.1 ;
  computeTransformationMatrix(rC,pC,yC,pC_w);
  std::cout << RT  << '\n';
  float rO = -0.028269 , pO = -0.099866 , yO = -0.870504 ;
  computeTransformationMatrix(rO,pO,yO,pO_w);
  std::cout << RT  << '\n';
}
