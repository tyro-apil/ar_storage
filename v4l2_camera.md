# Using v4l2_camera package
In Ubuntu 22.04 for ROS-humble

## Installation
```
sudo apt install ros-${ROS_DISTRO}-v4l2-camera
```

## Parameter files
Run v4l2_camera, and dump the parameters in yaml file like ...
```
ros2 run v4l2_camera v4l2_camera_node
```
```
ros2 param dump /v4l2_camera > <path_to_yaml_file>
```
**Parameter file example**
```
/**:
  ros__parameters:
    image_size:
      - 640
      - 480
    output_encoding: bgr8
    camera_frame_id: camera_optical_frame
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
    video_device: /dev/video2   # Change according to your device id
    camera_info_url: "package://silo/config/c270.yaml"
```
**Camera info file example**
```
image_width: 640
image_height: 480
camera_name: c270
camera_matrix:
  rows: 3
  cols: 3
  data: [720.8, 0.000000, 290.67425, 0.000000, 713.35, 195.11265, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0126, 0.8867, 0.0030, 0.0001507, -3.0664] # [k1, k2, t1, t2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000, 0.00, 0.00, 0.00, 1.000, 0.00, 0.00, 0.00, 1.00]
projection_matrix:
  rows: 3
  cols: 4
  data: [720.8, 0.000000, 290.67425, 0.000000, 0.000000, 713.35, 195.11265, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```

## Running node
```
ros2 run v4l2_camera v4l2_camera_node --ros-args --params-file <path_to_parameter_file>
```

## Pitfalls encountered
1. **Camera_info file** should have a**ll the elements** as shown in example. ***Missing any one (except camera_name)*** results in error.
2. Incorrect format for camera_info_url  
    If camera_info file inside a ros2 package:
    ```
    camera_info_url: 'package://<package_name>/<path_to_camera_info_file>'
    ```
    If camera_info file is given as file system path:
    ```
    camera_info_url: 'file://<absolute_path_to_camera_info_file>
    ```
    ***Do not forget to add 'file://...' before giving absolute path***

## Why I used <**v4l2**> instead of <**usb_cam**>?
Although ***usb_cam*** has **good documentation** and more **customization options** in parameters files. It doesnot support ***bgr*** output encoding natively. And, **bgr** is default encoding of opencv package.

## Reference
1. Official repo: [gitlab_link](https://gitlab.com/boldhearts/ros2_v4l2_camera)
2. For camera_info file: [usb_cam package](https://github.com/ros-drivers/usb_cam/tree/ros2)