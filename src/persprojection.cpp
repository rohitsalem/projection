// Date: 26th September 2017
// Author: rsalem
// Purpose: Convert 3d world coordinates to image pixels
#include "geometry.h"

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
  Matrix44f cameraToWorld(#####################)
  Matrix44f worldToCamera = cameraToWorld.inverse();

  // define canvas width & height
  float canvasWidth = 2, canvasHeight =2 ;
  // define image height and width here
  uint32_t imageWidth = 512, imageHeight = 512;

  
}
