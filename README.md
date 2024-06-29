# Robocon 2024, Quang Ning Vietnam

## Automatic Robot: Silo Detection | Silo Selection

### How to use this package

1. Since we are using Raspberry Pi Camerav3 and sending image frames using TCP socket, so make sure Pi and dev machine are **connected in same WiFi**.
    ```
    # For ssh using nmap scan and finding IP address using Pi's MAC address
    # In dev machine
    ~/.find_rpi2.sh
    ```

2. In Pi, open tcp_client.py (uses Picamera2 module)
    ```
    cd ~/work/picam
    ./tcp_client.py
    ```

3. Publish map->base_link transform for knowing distances to each silo

4. Run the top level launch file **silo.launch.py**
    ```
    ros2 launch silo silo.launch.py
    ```

### Errors Faced
1. If the TCP port of dev machine is busy, you can kill the port. So, that Pi can connect.
    ```
    # Find the process id (PID)
    sudo lsof -nP -iTCP:12345 -sTCP:LISTEN

    # Kill the process holding the port
    kill -15 <PID>
    # For forceful killing:
    kill -9 <PID>
    ```