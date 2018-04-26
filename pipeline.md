## Data Generation:
### Get the following ROS packages:
1) https://github.com/rohitsalem/projection 
2) https://github.com/osrf/tensorflow_object_detector
3) https://github.com/Kukanani/vision_msgs 
4) https://github.com/rohitsalem/car_demo 

And place them in a catkin workspace, RUN `catkin_make` 

### Using Different gazebo worlds:

1) Simple gray background: RUN `roslaunch projection getbox_setpose.launch`, this will launch a gazebo empty world with pedestrian teleporting to random locations.
2) Mcity world: RUN ‘roslaunch projection train_mcity.launch`, this will launch a raceway world with pedestrian teleporting along with the moving car according to the predefined waypoints. 
3) Raceway world: RUN ‘roslaunch projection train_raceway.launch`, this will launch a raceway world with pedestrian teleporting along with the moving car according to the predefined waypoints.   
4) Citysim: RUN `roslaunch projection train_citysim.launch`. This launches the citysim world with pedestrian teleporting along with the moving car according to the predefined waypoints.  

### Record bag files and extract data
1) Navigate to `projection/bag_files` and record a rosbag: RUN ` rosbag record /ShowBoundingBox/filtered/bounding_box /ShowBoundingBox/filtered/image `
2) To extract the bag file data into images and corresponding csv_files for labels, RUN `rosrun projection bag_extract.py`. The data is stored in `projection/datasets`. 
    
## Training:
1) Download the repo tensorflow/models : https://github.com/tensorflow/models , We are going to use the object_detection module in `models/research` . And follow the installation instructions for the tensorflow object-detection API. 
2) Download this folder: [pedestrian](https://drive.google.com/drive/folders/1FqTiHwD9ODp_u5bm9kEiyH2aF_arf8-G?usp=sharing) and place it in `models/research` . This folder has sample configuration, label_map files and some pre-trained weights (SSD-coco). 
3) Copy the entire `datasets` folder from the `projection` package generated in step 2 (of recording bag files) into the folder(pedestrian) downloaded in step 2(of training). 
4) We need to generate TFrecord files which are binary files that store all the data and corresponding labels. This format is used by tensorflow to process input data, So in ‘models/research/pedestrian` RUN: `python generate_tfrecord.py --csv_input=datasets/csv_files/data.csv --output_path=train.record ` We will have the train.record file now. 
5) We need to configure paths in `pedestrian/ped_train.config` to point to the tf record file and also change the paths for loading the pre-trained model and  save current model checkpoints while training. 
 In `models/research/`, 
To use the pre-trained weights trained on COCO dataset using SSD, RUN:  `python object_detection/train.py --logtostderr --pipeline_config=pedestrian/ped_train.config --train_dir=pedestrian/weights/`
To use random weights RUN: `python object_detection/train.py --logtostderr --pipeline_config=pedestrian/ped_train_random_init.config --train_dir=pedestrian/weights/`  this will set the output directory for weights in `models/research/pedestrian/weights`. 
    
    Visualizing the Loss and other related graphs:
 Using tensorboard: In a different terminal RUN: `tensorboard --logdir={PATH TO WEIGHTS (example: /home/user/models/pedestrian/weights)}` , Check the status of training in browser. 
    
Exporting the trained graph
 The trained weights for the model need to be exported as a frozen graph for inference, the weights are stored in `pedestrian/weights/`. Check the steps to export the graph from the API OR follow the steps below: 
From `models/research`, RUN: 
    python object_detection/export_inference_graph.py \
             --input_type image_tensor \
                       --pipeline_config_path pedestrian/ped_train.config \
                            --trained_checkpoint_prefix pedestrian/weights/model.ckpt-7756{Replace with the corresponding number} \
                            --output_directory pedestrian/output_inference_graph

Place the trained graph in the tensorflow_object_detector package inside the ‘data/models’ folder and then configure the model paths in detect_ros.py to point to the new frozen graph. 

Evaluating new Worlds: 
 First, modify the paths in detect_ros.py in tensorflow_object_detector/scripts to point to the appropriate trained object detection model graph. 
 Choose one world: 
For raceway:  RUN ‘ roslaunch projection demo_raceway.launch’
For mcity: RUN ‘roslaunch projection demo_mcity.launch’
For citysim: RUN ‘roslaunch projection demo_citysim.launch’
For gray_bkg: RUN ‘roslaunch projection demo_getbox_setpose.launch’ 
Then RUN `rosrun projection validate_log.py “{name_of_the_log_file}”.csv`, this will store the output in ‘projection/validation_output’ folder. (Make sure you have modified the path in the detect_ros.py to point to the model that is targeted) 

Creating new waypoints:
Choose a world: 
Raceway : Run `roslaunch projection raceway.launch` 
Mcity     : Run `roslaunch projection mcity.launch` 
Citysim : Run `roslaunch projection citysim.launch` 
 Navigate to projection/waypoints_data/bag_files. Drive the car around in the any of the three worlds, Then RUN `rosbag record /prius/getPose`. This will record the position of the car in a bag file. 
RUN `rosrun projection bag_extract_waypoints.py {Name of the csv file to store waypoints}`
The waypoints are written to a csv file stored in `waypoints_data/waypoints`, change the csv filename in the launch file train_raceway.launch or train_mcity.launch or train_citysim.launch depending on the world that is chosen . This csv file is taken as a parameter input by the setPose_prius.py node to set the pose of prius.  



