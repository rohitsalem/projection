// Date: 26th September 2017
// Author: rsalem
// Purpose: Convert 3d world coordinates to image pixels
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <cmath>
#include "geometry.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"


float x_min, y_min, z_min, x_max, y_max, z_max;
Vec3f pWorldA, pWorldB, pWorldC, pWorldD, pWorldE, pWorldF, pWorldG, pWorldH;

// Object's world coordiantes along with roll putch and yaw :
Vec3f pO_w (4.5, 0, 0.2);
float rO = 0 , pO = 0 , yO = 0 ;

Matrix44f RT;
void computeTransformationMatrix(float r , float p, float y, Vec3f c)
{
  Matrix44f Rx(1, 0, 0, 0, 0, cos(r), -sin(r), 0, 0, sin(r), cos(r), 0, 0, 0, 0, 1);
  Matrix44f Ry(cos(p), 0, sin(p), 0, 0, 1, 0, 0, -sin(p), 0, cos(p), 0, 0, 0, 0, 1);
  Matrix44f Rz(cos(y), -sin(y), 0, 0, sin(y), cos(y), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  Matrix44f T(1,0,0,0,0,1,0,0,0,0,1,0,c[0],c[1],c[2],1);

  Matrix44f Rxy;
  Matrix44f Rxyz;

  Matrix44f::multiply(Rx,Ry,Rxy);
  Matrix44f::multiply(Rxy, Rz, Rxyz);
  Matrix44f::multiply( Rxyz, T, RT);

}

std::vector<double> data(6);
void subscriberCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Accessing data from the message
  data = msg->data;

  // storing the data in max and min corners
  x_min = data[0];
  y_min = data[1];
  z_min = data[2];

  x_max = data[0];
  y_max = data[1];
  z_max = data[2];

  // Vertices of cuboid with respect to the object's origin.
  Vec3f pA_o(x_min, y_min, z_min);
  Vec3f pB_o(-x_min, y_min, z_min);
  Vec3f pC_o(-x_min, -y_min, z_min);
  Vec3f pD_o(x_min, -y_min, z_min);
  Vec3f pE_o(x_max, y_max, z_max);
  Vec3f pF_o(-x_max, y_max, z_max);
  Vec3f pG_o(-x_max, -y_max, z_max);
  Vec3f pH_o(x_max, -y_max, z_max);

  // transformation matrix from objectToWorld
  computeTransformationMatrix(rO,pO,yO,pO_w);
  // transformation matrix from worldToObject
  Matrix44f worldToObject = RT.inverse();

  // compute the world coordinates of the corners of the box
  RT.Matrix44f::multVecMatrix(pA_o,pWorldA);
  RT.Matrix44f::multVecMatrix(pB_o,pWorldB);
  RT.Matrix44f::multVecMatrix(pC_o,pWorldC);
  RT.Matrix44f::multVecMatrix(pD_o,pWorldD);
  RT.Matrix44f::multVecMatrix(pE_o,pWorldE);
  RT.Matrix44f::multVecMatrix(pF_o,pWorldF);
  RT.Matrix44f::multVecMatrix(pG_o,pWorldG);
  RT.Matrix44f::multVecMatrix(pH_o,pWorldH);

  std::cout << pWorldA << '\n';
  std::cout << pWorldB << '\n';
  std::cout << pWorldC << '\n';
  std::cout << pWorldD << '\n';
  std::cout << pWorldE << '\n';
  std::cout << pWorldF << '\n';
  std::cout << pWorldG << '\n';
  std::cout << pWorldH << '\n';

}

int main (int argc, char **argv)
{
  ros::init(argc, argv , "perspectiveProjection");
  // NodeHandle for ros
  ros::NodeHandle nh;
  // subscriber for ros
  ros::Subscriber sub = nh.subscribe("corners", 1, subscriberCallback);
  // spin continuously
  ros::spin();

  }
