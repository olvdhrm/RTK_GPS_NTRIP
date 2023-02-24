# RTK_GPS_NTRIP
System built on ROS2 Humble distribution to utilize NTRIP for a RTK Global Navigation Satelite System (GNSS). 

RTCM corrections obtained through an NTRIP Client are passed through a ROS Topic and provided to the u-blox driver to obtain high accuracy in GPS/GNSS positioning. 

### 1. u-blox Driver 
u-blox driver by KumarRobotics is used and can be found here https://github.com/KumarRobotics/ublox.git. The current configuration set is utilized for a ZED F9P receiver acting as a rover. 

To activate the ublox node, run the following command in the terminal: `ros2 launch ublox_gps ublox_gps_node-launch.py`

### 2. NTRIP Client 
NTRIP Client package by LORD-MicroStrain is used and can be found here https://github.com/LORD-MicroStrain/ntrip_client.git. Prior to running the command below, provide the NTRIP credentials (for access, if required) in the ntrip_client_launch.py file including the host, port, mountpoint, username, and password.  

To activate the NTRIP Client node, run the following command in the terminal: `ros2 launch ntrip_client ntrip_client_launch.py`

### 3. /fix to /nmea Parser 
As the NTRIP Client requires an NMEA message, the /fix type message is parsed into an /nmea type message by running the fix2nmea executable. It can be executed by running the following command in the terminal: `ros2 run fix2nmea fix2nmea`

