#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os, shutil
import argparse

import rosbag
import math
from sensor_msgs.msg import NavSatFix

# WGS-84 geodetic constants
a = 6378137.0;      # WGS-84 Earth semimajor axis (m)
b = 6356752.314245; # WGS-84 Earth semiminor axis (m)

def GeodeticToEnu(meas, datum):
    xyz_ecef = GeodeticToEcef(meas)
    return EcefToEnu(xyz_ecef, datum)

def GeodeticToEcef(meas):
    lat = meas[0]
    lon = meas[1]
    h = meas[2]
    f = (a - b) / a
    e_sq = f * (2 - f)
    
    lambda_val = DegreeToRadian(lat)
    phi = DegreeToRadian(lon)
    s = math.sin(lambda_val)
    N = a / math.sqrt(1 - e_sq * s * s)
    sin_lambda = math.sin(lambda_val)
    cos_lambda = math.cos(lambda_val)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    xyz_ecef = [0, 0, 0]
    xyz_ecef[0] = (h + N) * cos_lambda * cos_phi
    xyz_ecef[1] = (h + N) * cos_lambda * sin_phi
    xyz_ecef[2] = (h + (1 - e_sq) * N) * sin_lambda
    return xyz_ecef
  
def DegreeToRadian(angle):
    return math.pi * angle / 180.0

def EcefToEnu(xyz_ecef, datum):
    f = (a - b) / a
    e_sq = f * (2 - f)
    
    lambda_val = math.radians(datum[0])
    phi = math.radians(datum[1])
    s = math.sin(lambda_val)
    N = a / math.sqrt(1 - e_sq * s * s)
    sin_lambda = math.sin(lambda_val)
    cos_lambda = math.cos(lambda_val)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    x0 = (datum[2] + N) * cos_lambda * cos_phi
    y0 = (datum[2] + N) * cos_lambda * sin_phi
    z0 = (datum[2] + (1 - e_sq) * N) * sin_lambda
    xd = xyz_ecef[0] - x0
    yd = xyz_ecef[1] - y0
    zd = xyz_ecef[2] - z0
    
    xyz_enu = [0, 0, 0]
    xyz_enu[0] = -sin_phi * xd + cos_phi * yd
    xyz_enu[1] = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    xyz_enu[2] = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
    return xyz_enu

def main():
    for i in range(1,2):
        dir = "/home/wl/Desktop/husky/"
        bag = rosbag.Bag(dir + "husky_trail4.bag", "r")
        output_dir = dir + "husky_trail4_gnss.txt"
        fd = os.open(output_dir, os.O_RDWR|os.O_CREAT)
        os.write(fd, ("# timestamp(s) tx ty tz qx qy qz qw\n").encode())
        print(output_dir + "...      ", end='', flush=True)
        datum_set = False
        count = 0
        for topic, msg, t in bag.read_messages(topics=["/fix"]):
            # Get LLT
            llt = [msg.latitude, msg.longitude, msg.altitude]
            if datum_set == False:
                datum = llt
                datum_set = True

            # Get ENU
            xyz = GeodeticToEnu(llt, datum)
            time = str(msg.header.stamp.to_sec()) + " "
            pos = str(xyz[0]) + " " + str(xyz[1]) + " " + str(xyz[2]) + " "
            quat = str("0 0 0 1\n")
            os.write(fd, time.encode())
            os.write(fd, pos.encode())
            os.write(fd, quat.encode())
            print('\b\b\b\b\b\b'f'{count:06}', end='', flush=True)

            count += 1


        os.close(fd)
        bag.close()
        print()

    return

if __name__ == '__main__':
    main()
