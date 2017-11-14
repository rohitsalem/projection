// Date 23rd October 2017
// Author: rsalem
// Purpose: Set object Pose with respect a reference frame example camera or car
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <cmath>
#include "geometry.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"

class setPosePerson
{
public:
  setPosePerson()
  {
    sub = nh.subscribe("/prius/getPose", 1, &setPosePerson::subscriberCallback, this);
    pub = nh.advertise<geometry_msgs::Pose>("/person_walking/SetObjectPose",1);
    counter_pub = nh.advertise<std_msgs::Int32>("/setPose/counter",1);
  }

  float x_world, y_world, z_world, roll_world, pitch_world, yaw_world ; // world coordinates of the reference frame
  Matrix44f RT;
  Vec3f po_w;
  std::vector<double> data;
  geometry_msgs::Pose pose;
  std_msgs::Int32 count; // counter updates when new pose is set
  void computeTransformationMatrix(float r , float p, float y, Vec3f c)
  {
    Matrix44f Rx(1, 0, 0, 0, 0, cos(r), -sin(r), 0, 0, sin(r), cos(r), 0, 0, 0, 0, 1);
    Matrix44f Ry(cos(p), 0, sin(-p), 0, 0, 1, 0, 0, -sin(-p), 0, cos(p), 0, 0, 0, 0, 1);
    Matrix44f Rz(cos(y), -sin(-y), 0, 0, sin(-y), cos(y), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    Matrix44f T(1,0,0,0,0,1,0,0,0,0,1,0,c[0],c[1],c[2],1);

    Matrix44f Rxy;
    Matrix44f Rxyz;

    Matrix44f::multiply(Rx,Ry,Rxy); // Saves Rx*Ry in Rxy
    Matrix44f::multiply(Rxy, Rz, Rxyz); // Saves Rxy*Rz in Rxyz
    Matrix44f::multiply( Rxyz, T, RT); // Saves Rxyz*T in RT
    // Matrix44f::multiply( Rx, T, RT); // Saves Rxyz*T in RT
    // RT = T;
  }

  double Rand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  void subscriberCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    data = msg->data;
    x_world = data[0]; // World coordinates of the reference frame
    y_world = data[1];
    z_world = data[2];
    roll_world = data[3];
    pitch_world = data[4];
    yaw_world = data[5];

    // reference frame's world coordiantes along with roll putch and yaw :
    Vec3f pr_w (x_world, y_world, z_world);
    // transformation matrix from objectToWorld
    computeTransformationMatrix(roll_world, pitch_world, yaw_world, pr_w);
    // transformation matrix from worldToObject
    Matrix44f worldToObject = RT.inverse();
    if (i==0) // to set starting x value
    {
    x = 7.5 ;
  }

    // Origin of the object with respect to reference frame
    if (i%300 ==0)
    {
     x = setPosePerson::Rand(6.5,8.5);
     y = setPosePerson::Rand(-1.0,1.0);
     roll = setPosePerson::Rand(-0.02,0.02);
     pitch = setPosePerson::Rand(-0.02,0.02);
     yaw = setPosePerson::Rand(-3.4,3.4);

}
  //  std::cout << i <<" " << x << " " << y << '\n';
    Vec3f po_r (x, y ,0.05);

    RT.Matrix44f::multVecMatrix(po_r,po_w);

    pose.position.x = po_w[0];
    pose.position.y = po_w[1];
    pose.position.z = po_w[2];
    pose.orientation.x = roll;
    pose.orientation.y = pitch;
    pose.orientation.z = yaw;
    pub.publish(pose);
    count.data = count.data+1;
    counter_pub.publish(count);

  i +=1;

  }
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher counter_pub;
  double x,y,z,roll,pitch,yaw;
  int i =0;
};
int main (int argc, char **argv)
{
  ros::init(argc, argv , "setPosePerson");

  setPosePerson object;
  // spin continuously
  ros::spin();
  return 0;
}
