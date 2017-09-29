// Date: 26th September 2017
// Author: rsalem
// Purpose: Convert 3d world coordinates to image pixels
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <cmath>

#include "geometry.h"
float h = 1.9;
float b = 0.55;
float w = 0.8;

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

bool computePixelCoordinates(
    const Vec3f &pWorld,
    const Matrix44f &worldToCamera,
    const float &b,
    const float &l,
    const float &t,
    const float &r,
    const float &near,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    Vec2i &pRaster)
{
    Vec3f pCamera;
    worldToCamera.multVecMatrix(pWorld, pCamera);
    Vec2f pScreen;
    pScreen.x = pCamera.x / -pCamera.z * near;
    pScreen.y = pCamera.y / -pCamera.z * near;

    Vec2f pNDC;
    pNDC.x = (pScreen.x + r) / (2 * r);
    pNDC.y = (pScreen.y + t) / (2 * t);
    pRaster.x = (int)(pNDC.x * imageWidth);
    pRaster.y = (int)((1 - pNDC.y) * imageHeight);

    bool visible = true;
    if (pScreen.x < l || pScreen.x > r || pScreen.y < b || pScreen.y > t)
        visible = false;

    return visible;
}

float nearClippingPlane = 0.1;
float farClipingPlane = 100;

uint32_t imageWidth  = 512 ;
uint32_t imageHeight = 512 ;

int main (int agrc, char **argv)
{
  // define canvas width & height
  float canvasWidth = 2, canvasHeight = 2 ;
  // define image height and width here
  uint32_t imageWidth = 512, imageHeight = 512;
  float rC = 0 , pC = 0.15 , yC = 0.1 ;

  // transformation matrix from cameraToWorld
  computeTransformationMatrix(rC,pC,yC,pC_w);
  // transformation matrix from worldToCamera

  Matrix44f worldToCamera = RT.inverse();
  // std::cout << RT << '\n';
  // std::cout << worldToCamera << '\n';

  float rO = -0.028269 , pO = -0.099866 , yO = -0.870504 ;
  // float rO = 0, pO = 0, yO= 0;
  // transformation matrix from objectToWorld
  computeTransformationMatrix(rO,pO,yO,pO_w);
  // transformation matrix from worldToObject
  Matrix44f worldToObject = RT.inverse();

  Vec3f pWorldA, pWorldB, pWorldC, pWorldD, pWorldE, pWorldF, pWorldG, pWorldH;
  // compute the world coordinates of the corners of the box
  RT.Matrix44f::multVecMatrix(pA_o,pWorldA);
  RT.Matrix44f::multVecMatrix(pB_o,pWorldB);
  RT.Matrix44f::multVecMatrix(pC_o,pWorldC);
  RT.Matrix44f::multVecMatrix(pD_o,pWorldD);
  RT.Matrix44f::multVecMatrix(pE_o,pWorldE);
  RT.Matrix44f::multVecMatrix(pF_o,pWorldF);
  RT.Matrix44f::multVecMatrix(pG_o,pWorldG);
  RT.Matrix44f::multVecMatrix(pH_o,pWorldH);

  // std::cout << pWorldA << '\n';
  // std::cout << pWorldB << '\n';
  // std::cout << pWorldC << '\n';
  // std::cout << pWorldD << '\n';
  // std::cout << pWorldE << '\n';
  // std::cout << pWorldF << '\n';
  // std::cout << pWorldG << '\n';
  // std::cout << pWorldH << '\n';

  Vec2i vARaster, vBRaster, vCRaster, vDRaster, vERaster, vFRaster, vGRaster, vHRaster;
  computePixelCoordinates(pWorldA, vARaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldB, vBRaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldC, vCRaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldD, vDRaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldE, vERaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldF, vFRaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldG, vGRaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  computePixelCoordinates(pWorldH, vHRaster, worldToCamera, canvasWidth, canvasHeight, imageWidth, imageHeight);
  std::cout << vARaster << '\n';
  std::cout << vBRaster << '\n';
  std::cout << vCRaster << '\n';
  std::cout << vDRaster << '\n';
  std::cout << vFRaster << '\n';
  std::cout << vFRaster << '\n';
  std::cout << vGRaster << '\n';
  std::cout << vHRaster << '\n';

  }
