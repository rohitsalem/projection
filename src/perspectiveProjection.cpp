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


class persProjection
{
public:
  persProjection()
  {
    pub = nh.advertise<std_msgs::Float64MultiArray>("objectBoxWorldCoordinates",1);
    sub = nh.subscribe("corners", 1, &persProjection::subscriberCallback, this);
  }

  float x_min, y_min, z_min, x_max, y_max, z_max;
  float x_world, y_world, z_world, roll_world, pitch_world, yaw_world ; // world coordiantes of the object
  Vec3f pWorldA, pWorldB, pWorldC, pWorldD, pWorldE, pWorldF, pWorldG, pWorldH;
  std::vector<double> data;


  Matrix44f RT;
  std_msgs::Float64MultiArray worldArr;

  void computeTransformationMatrix(float r , float p, float y, Vec3f c)
  {
    Matrix44f Rx(1, 0, 0, 0, 0, cos(r), -sin(-r), 0, 0, sin(-r), cos(r), 0, 0, 0, 0, 1);
    Matrix44f Ry(cos(p), 0, sin(p), 0, 0, 1, 0, 0, -sin(p), 0, cos(p), 0, 0, 0, 0, 1);
    Matrix44f Rz(cos(y), -sin(y), 0, 0, sin(y), cos(y), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    Matrix44f T(1,0,0,0,0,1,0,0,0,0,1,0,c[0],c[1],c[2],1);

    Matrix44f Rxy;
    Matrix44f Rxyz;

    Matrix44f::multiply(Rx,Ry,Rxy); // Saves Rx*Ry in Rxy
    Matrix44f::multiply(Rxy, Rz, Rxyz); // Saves Rxy*Rz in Rxyz
    Matrix44f::multiply( Rxyz, T, RT); // Saves Rxyz*T in RT

  }

  void subscriberCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    // Accessing data from the message
    data = msg->data;
    // storing the data in max and min corners
    x_min = data[0];  // Min corners
    y_min = data[1];
    z_min = data[2];
    x_max = data[3]; // Max corners
    y_max = data[4];
    z_max = data[5];
    x_world = data[6]; // World coordinates
    y_world = data[7];
    z_world = data[8];
    roll_world = data[9];
    pitch_world = data[10];
    yaw_world = data[11];

    // Vertices of cuboid with respect to the object's origin.
    Vec3f pA_o(x_min, y_min, z_min);
    Vec3f pB_o(-x_min, y_min, z_min);
    Vec3f pC_o(-x_min, -y_min, z_min);
    Vec3f pD_o(x_min, -y_min, z_min);
    Vec3f pE_o(x_max, y_max, z_max);
    Vec3f pF_o(-x_max, y_max, z_max);
    Vec3f pG_o(-x_max, -y_max, z_max);
    Vec3f pH_o(x_max, -y_max, z_max);

    // Object's world coordiantes along with roll putch and yaw :
    Vec3f pO_w (x_world, y_world, z_world);
    // transformation matrix from objectToWorld
    computeTransformationMatrix(roll_world, pitch_world, yaw_world, pO_w);
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

    // Transfer into data msg to publish
    worldArr.data.clear();
    worldArr.data.push_back(pWorldA[0]);
    worldArr.data.push_back(pWorldA[1]);
    worldArr.data.push_back(pWorldA[2]);
    worldArr.data.push_back(pWorldB[0]);
    worldArr.data.push_back(pWorldB[1]);
    worldArr.data.push_back(pWorldB[2]);
    worldArr.data.push_back(pWorldC[0]);
    worldArr.data.push_back(pWorldC[1]);
    worldArr.data.push_back(pWorldC[2]);
    worldArr.data.push_back(pWorldD[0]);
    worldArr.data.push_back(pWorldD[1]);
    worldArr.data.push_back(pWorldD[2]);
    worldArr.data.push_back(pWorldE[0]);
    worldArr.data.push_back(pWorldE[1]);
    worldArr.data.push_back(pWorldE[2]);
    worldArr.data.push_back(pWorldF[0]);
    worldArr.data.push_back(pWorldF[1]);
    worldArr.data.push_back(pWorldF[2]);
    worldArr.data.push_back(pWorldG[0]);
    worldArr.data.push_back(pWorldG[1]);
    worldArr.data.push_back(pWorldG[2]);
    worldArr.data.push_back(pWorldH[0]);
    worldArr.data.push_back(pWorldH[1]);
    worldArr.data.push_back(pWorldH[2]);
    //Publish message
    pub.publish(worldArr);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;

};
int main (int argc, char **argv)
{
  ros::init(argc, argv , "perspectiveProjection");

  persProjection object;
  // spin continuously
  ros::spin();
  return 0;
}
