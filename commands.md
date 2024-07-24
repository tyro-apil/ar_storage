## CLI commands

1. Run state_estimation_node separately
    ```
    ros2 run silo state_estimation_node --ros-args --params-file ~/main_ws/src/robot/config/common.yaml --params-file ~/main_ws/src/silo/config/camera_info.yaml --params-file ~/main_ws/src/silo/config/silo.yaml -r __ns:=/silo
    ```