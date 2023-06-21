#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os, shutil
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Imu

def main():
    for i in range(1,9):
        dir = "/home/rpng/datasets/rpng_plane/"
        bag = rosbag.Bag(dir + "table_0" + str(i) + ".bag", "r")
        output_dir = dir + "table_0" + str(i) + "_imus"
        if os.path.isdir(output_dir):
            shutil.rmtree(output_dir)
        os.mkdir(output_dir)
        count = 0
        print(output_dir + "...      ", end='', flush=True)
        for topic, msg, t in bag.read_messages(topics=["/d455/imu"]):
            fd = os.open(output_dir + "/" + str(t.to_nsec()) + ".txt", os.O_RDWR|os.O_CREAT)
            time = str(msg.header.stamp.to_sec()) + ","
            ang_vel = str(msg.angular_velocity.x) + "," + str(msg.angular_velocity.y) + "," + str(msg.angular_velocity.z) + ","
            lin_acc = str(msg.linear_acceleration.x) + "," + str(msg.linear_acceleration.y) + "," + str(msg.linear_acceleration.z) + ","
            os.write(fd, time.encode())
            os.write(fd, ang_vel.encode())
            os.write(fd, lin_acc.encode())
            os.close(fd)
            print('\b\b\b\b\b\b'f'{count:06}', end='', flush=True)

            count += 1

        bag.close()
        print()

    return

if __name__ == '__main__':
    main()
