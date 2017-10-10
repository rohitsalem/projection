# projection

**Package for Auto generation of Bounding boxes in Gazebo for gazebo Models**
Requirement: ROS Kinetic, Gazebo-8, gazebo_ros and ROS message_filter packages.

Usage: `roslaunch projection getboxes.launch`

This will open Rviz and Gazebo, 
* The Image with the 2D bounding box can be found on ros topic `/ShowBoundingBox/image_raw `
* The minimum and maximum corners of the 2D Bounding Box is on ROS topic `/pixels` : The message is in the order (x_min, y_min, x_max, y_max)
* The object(person) in the default world launched can be moved around, and the corresponding Bounding Boxes can be seen overlayed on the image in Rviz. 

To Set random poses to the object and see the corresponding Bounding Boxes:
RUN `roslaunch projection getboxes_setpose.launch`
* This will launch a node(setPose.py) to publish random poses to the setPosesPlugin attached to the Model.  
