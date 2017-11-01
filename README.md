# projection

## Package for Auto generation of Bounding boxes in Gazebo for gazebo Models

**Requirement:**  ROS Kinetic, Gazebo-8, and
  ROS packages:
* gazebo_ros 
* message_filter
* [vision_msgs](https://github.com/rohitsalem/vision_msgs)

**Recommended ROS Packages:** 
* [car_demo](https://github.com/rohitsalem/car_demo) 
* [tensorflow_object_detector](https://github.com/osrf/tensorflow_object_detector)
* [yolo_object_detector](https://github.com/osrf/yolo_object_detector)

## If Using Docker (RECOMMENDED): 
**Requirements:** 
* An X server
* [Docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/) 
* [Nvidia-Docker](https://github.com/NVIDIA/nvidia-docker)
* Place this package and the above recommended packages (if you want to ckeck out the validation example with tensorflow_detector) in `workspace/src` folder on your local machine.
* Replace the `WORKSPACE` variable in `docker/run_demo.bash` to point to your workspace in your local machine.  
* Then, RUN:
```
cd projection/docker
bash run_demo.bash
```
[//]: # (Image References)
[gif1]: ./screenshots-bbox/getbox.gif
[gif2]: ./screenshots-bbox/getbox_setpose.gif
[gif3]: ./screenshots-bbox/demo.gif
[gif4]: ./screenshots-bbox/demo_setpose.gif

**To check out the basic version**, like the one in the gif below,
![alt text][gif1]
* Place this packge in your workspace's src folder (Ignore this step if you followed previous steps for using docker).
* RUN `catkin_make`.
* RUN `roslaunch projection getbox.launch`.

This will open Rviz and Gazebo, 
* The Image with the 2D bounding box can be found on ros topic `/ShowBoundingBox/image_raw `
* The minimum and maximum corners of the 2D Bounding Box is on ROS topic `/pixels` : The message is in the order (x_min, y_min, x_max, y_max)
* The object (person) in the default world launched can be moved around, and the corresponding Bounding Boxes can be seen overlayed on the image in Rviz. 

**To set random poses to the object and see the corresponding Bounding Boxes,**
RUN `roslaunch projection getbox_setpose.launch`
* This will launch a node(setPose.py) to publish random poses to the setPosesPlugin attached to the Model.  Like this:
![alt text][gif2]

### To use the model with a prius car and simulated city, along with tensoflow object recoginiton  
Like this:

![alt text][gif3]

* Make sure that you have tensorflow installed, for tensoflow installation check [here](https://www.tensorflow.org/install/install_linux).
* Get all of recommended the recommended ROS packages along with this package and place them in your workspace. (Ignore this step if you followed previous steps for using docker). 
* RUN `catkin_make`.
* RUN `roslaunch projection demo.launch`, this will launch the mcity world with prius car and a person (object) by default and you can move the car using your joystick or `WASD` on your keyboard, The person model will move randomly along with the car. 
* To get the accuracy score of the tensorflow detection, RUN `rosrun projection validate.py`.

**To set the prius to follow a predifined path** like this:
![alt text][gif4]

* RUN : `roslaunch projection demo_setpose.launch`, this will also run `setPose_prius.py` node to set pose of the prius according to the poses in the csv [file](./data/waypoints.csv).
* Then again, to get the accuracy of the tensorflow detection, RUN `rosrun projection validate.py`.

