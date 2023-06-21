#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os, shutil
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    for i in range(1,9):
        dir = "/home/rpng/datasets/rpng_plane/"
        bag = rosbag.Bag(dir + "table_0" + str(i) + ".bag", "r")
        output_dir = dir + "table_0" + str(i) + "_imgs"
        if os.path.isdir(output_dir):
            shutil.rmtree(output_dir)
        os.mkdir(output_dir)
        bridge = CvBridge()
        count = 0
        print(output_dir + "...    ", end='', flush=True)
        for topic, msg, t in bag.read_messages(topics=["/d455/color/image_raw"]):
            # Write image
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(os.path.join(output_dir, "%d.png" % t.to_nsec()), cv_img)
            # Also write the timestamp!!!! t != msg.header.stamp !!!!!!!!!!!!!
            fd = os.open(output_dir + "/" + str(t.to_nsec()) + ".txt", os.O_RDWR|os.O_CREAT)
            time = str(msg.header.stamp.to_sec())
            os.write(fd, time.encode())
            os.close(fd)
            print('\b\b\b\b'f'{count:04}', end='', flush=True)

            count += 1

        bag.close()
        print()

    return

if __name__ == '__main__':
    main()
