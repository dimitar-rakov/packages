# Visual Features Extraction

ROS/indigo package for visual features extraction

## Overview

This package contains the implementation of 2 different features extraction method. First one is based on coloured blobs. The second one is based on aruco markers


## Usage

The node from this package can be run with the aid of the launch files from the _launch_ directory. 
The following example lstart a visual_extraction_node:  
```roslaunch visual_features_extractor visual_features_extractor.launch```  


## Parameters

The package consists of multiple parameters, which can be modifyed from launch file. Here is given the list of them:
```
  base_name"                     default="visual_fetures_extractor"/>
  raw_images_topic"              default="/usb_cam/image_raw"/>
  camera_name"                   default="eye_in_hand"/>
  using_sim_features"            default="false"/>
  using_extended_features"       default="true"/>
  using_symmetrical_features"    default="true"/>
  extended_features_var"         default="1.2"/>
  using_colored_blobs"           default="false"/>
  contour_area_threshold"        default="25.0"/>
  blobs_color_ranges"            default="[ {h_min: 170, h_max: 10}, {h_min: 40, h_max: 80}, {h_min: 100, h_max: 125}, {h_min: 130, h_max: 165} ]"/>
  arucos_param"                  default="[ {id: 110, size: 0.04}, {id: 120, size: 0.04}, {id: 130, size: 0.04}, {id: 140, size: 0.04} ]"/>
  camera_param_file"             default="$(find usb_cam)/eye_in_hand_640x480.yaml"/>

```



