#include <iostream>

struct Vector2i {
  int x, y;
};

struct Vector3i {
  int x, y, z;
};

int main() {

  // lets assume 640x480 res.
  // Our "eye" is where we are viewing from, which
  // is about 800 pixels towards me and in the center of
  // the screen.
  Vector3i eye = {320, 240, 800};
  
  // This is the point that we're projecting onto
  // our 2D plane.
  Vector3i P = {600, 200, 1000};
  
  // This will be the 2D coords of our perspective projection.
  Vector2i S;
  
  // ---------------------------------------- 
  // Formula to solve Sx
  // ----------------------------------------
  // Ez = distance from eye to the center of the screen
  // Ex = X coordinate of the eye
  // Px = X coordinate of the 3D point
  // Pz = Z coordinate of the 3D point
  //
    //              Ez*(Px-Ex)
    // Sx  = -----------------------  + Ex   
  //          Ez+Pz 
  S.x = (eye.z * (P.x-eye.x)) / (eye.z + P.z) + eye.x;
  
  
  // ---------------------------------------- 
  // Formula to solve Sy
  // ----------------------------------------
  // Ez = distance from eye to the center of the screen
  // Ey = Y coordinate of the eye 
  // Py = Y coordinate of the 3D point
  // Pz = Z coordinate of the 3D point  
  //
    //            Ez*(Py-Ey)
    // Sy  = -------------------  + Ey      
    //              Ez+Pz
  S.y = (eye.z * (P.y-eye.y)) / (eye.z + P.z) + eye.y;
  
  std::cout << "Result of projection.\nx: " << S.x << std::endl
        << "y: " << S.y;

  return 0;
}
