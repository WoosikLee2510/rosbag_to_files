# rosbag_to_files
Extracts sensor measurements into files:
* imu: time, ang vel, lin acc
* cam: time, image
* gps: time, position (rotation will be set to 0 0 0 1) - Also automatically convert LLT to ENU coordinate.

# how to use
Directly change the path to a rosbag and run!
