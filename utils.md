## Testing 

1. To see full state prediction using partial state information, need to publish **/aligned_silo** of **std_msgs/msg/UInt8**:
    ```
    ros2 topic pub --rate 30 /aligned_silo std_msgs/msg/UInt8 '{"data":3}'
    ```