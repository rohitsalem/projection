# projection

**Package for Auto generation of Bounding boxes in Gazebo for gazebo Models**
Requirement: ROS Kinetic, Gazebo-8, gazebo_ros and ROS message_filter packages.

[//]: # (Image References)
[image1]: ./screenshots-bbox/box.png
**To check out the basic version**, like the one in the image below, RUN `roslaunch projection getbox.launch`

![alt text][image1]

This will open Rviz and Gazebo, 
* The Image with the 2D bounding box can be found on ros topic `/ShowBoundingBox/image_raw `
* The minimum and maximum corners of the 2D Bounding Box is on ROS topic `/pixels` : The message is in the order (x_min, y_min, x_max, y_max)
* The object (person) in the default world launched can be moved around, and the corresponding Bounding Boxes can be seen overlayed on the image in Rviz. 

**To set random poses to the object and see the corresponding Bounding Boxes,**
RUN `roslaunch projection getbox_setpose.launch`
* This will launch a node(setPose.py) to publish random poses to the setPosesPlugin attached to the Model.  
