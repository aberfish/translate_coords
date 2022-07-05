# translate_coords
ROS package containing nodes for converting from 2d coordinates to 3d RTAB_map coordinates.

- [translate_coords](#translate_coords)
  - [Nodes](#nodes)
    - [cam_to_depthcoords.py](#cam_to_depthcoordspy)
      - [Subscribes:](#subscribes)
      - [Publishes:](#publishes)
      - [Parameters:](#parameters)
      - [Usage:](#usage)
    - [depth_to_mapcoords.py](#depth_to_mapcoordspy)
      - [Subscribes:](#subscribes-1)
      - [Publishes:](#publishes-1)
      - [Parameters:](#parameters-1)
      - [Usage:](#usage-1)
  - [Setup](#setup)
  - [Launch Files](#launch-files)

## Nodes
### cam_to_depthcoords.py
Converts 2d pixel coordinates from depth camera colour images to realworld 3d coordinates measured in metres, where 0,0,0 is **WHERE**.

#### Subscribes:
 - */camera/depth/camera_info* (sensor_msgs.CameraInfo): Camera intrinsics data
 - */camera/aligned_depth_to_color/image_raw* (sensor_msgs.Image): Depth image
 - */input_coords* (geometry_msgs.Point): Input pixel location to be converted. 'z' is ignored

#### Publishes:
 - */realworld_coords* (geometry_msgs.Point): Position in metres, relative to the camera, of the input pixel location

#### Parameters:
*NONE*

#### Usage:
```rosrun translate_coords cam_to_depthcoords.py```

### depth_to_mapcoords.py
Converts 3d realworld coordinates measured in metres relative to the camera to RTAB_map coordinates.

#### Subscribes:
*NONE*
 <!-- - */camera/depth/camera_info* (sensor_msgs.CameraInfo): Camera intrinsics data -->

#### Publishes:
*NONE*
 <!-- - */realworld_coords* (geometry_msgs.Point): Position in metres, relative to the camera, of the input pixel location -->

#### Parameters:
*NONE*

#### Usage:
```rosrun translate_coords depth_to_mapcoords.py```

## Setup
Place contents of repository in the directory ```<ros_workspace_dir>/src/translate_coords```. Build and source your ROS workspace.

## Launch Files
<!-- ### tracker_debug
Example usage: 

```roslaunch tango_tracker tracker_debug.launch marker_size:=5 rosbag_path:="/home/rose/Documents/TangoProject/2022-06-06-14-05-29.bag"```

#### Arguments
 - *marker_size* (float): Measuerment in **centimeters** of aruco marker width/height
 - *rosbag_path* (str): Absolute path to a rosbag containing the */camera/color/raw_image* topic -->