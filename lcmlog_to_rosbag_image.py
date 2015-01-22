#!/usr/bin/env python
import argparse
import yaml

import numpy as np

import lcm
from mav import image_t

import rosbag
from sensor_msgs.msg import CompressedImage, CameraInfo


def main():
    parser = argparse.ArgumentParser(
        description="Convert an LCM log to a ROS bag (mono/stereo images only).")
    parser.add_argument('lcm_file', help='Input LCM log.', action='store')
    parser.add_argument('left_img_channel', help='LCM channel for left image.')
    parser.add_argument('left_camera_yml',
                        help='Image calibration YAML file from ROS calibrator')
    parser.add_argument('--right_img_channel', help='LCM channel for right image.',
                        action='append', dest='lcm_channels')
    parser.add_argument('--right_camera_yml',
                        help='Image calibration YAML file from ROS calibrator',
                        action='append', dest='yml_files')
    args = parser.parse_args()

    args.lcm_channels.append(args.left_img_channel)
    args.yml_files.append(args.left_camera_yml)

    if len(args.lcm_channels) != len(args.yml_files):
        print "LCM channel-YAML file mismatch!"

    print "Converting images in %s to ROS bag file..." % (args.lcm_file)

    log = lcm.EventLog(args.lcm_file, 'r')
    bag = rosbag.Bag(args.lcm_file + '.images.bag', 'w')

    # Read in YAML files.
    yml = []
    for y in args.yml_files:
        yml.append(yaml.load(file(y)))

    try:
        count = 0
        for event in log:
            for ii in range(len(args.lcm_channels)):
                l = args.lcm_channels[ii]
                y = yml[ii]

                if event.channel == l:
                    lcm_msg = image_t.decode(event.data)

                    # Fill in image.
                    if lcm_msg.pixelformat != image_t.PIXEL_FORMAT_MJPEG:
                        print "Encountered non-MJPEG compressed image. Skipping..."
                        continue

                    ros_msg = CompressedImage()
                    ros_msg.header.seq = event.eventnum

                    secs_float = float(lcm_msg.utime)/1e6
                    nsecs_float = (secs_float - np.floor(secs_float)) * 1e9
                    ros_msg.header.stamp.secs = np.uint32(np.floor(secs_float))
                    ros_msg.header.stamp.nsecs = np.uint32(np.floor(nsecs_float))

                    ros_msg.format = 'jpeg'

                    ros_msg.data = lcm_msg.data

                    # Fill in camera info
                    camera_info = CameraInfo()
                    camera_info.header = ros_msg.header
                    camera_info.height = lcm_msg.height
                    camera_info.width = lcm_msg.width

                    if y["distortion_model"] != "plumb_bob":
                        print "Encountered non-supported distorion model %s. Skipping..." % y["distortion_model"]
                        continue

                    camera_info.distortion_model = y["distortion_model"]
                    camera_info.D = y["distortion_coefficients"]['data']
                    camera_info.K = y["camera_matrix"]['data']
                    camera_info.R = y["rectification_matrix"]['data']
                    camera_info.P = y["projection_matrix"]['data']
                    camera_info.binning_x = 0
                    camera_info.binning_y = 0
                    # camera_info.roi = sensor_msgs.msg.RegionOfInterest() 

                    bag.write("/camera/" + l + "/image_raw/compressed", ros_msg)
                    bag.write("/camera/" + l + "/camera_info", camera_info)

                    count += 1

                    if count % 100 == 0:
                        print "Wrote %i events" % count
    finally:
        log.close()
        bag.close()

    print("Done.")

if __name__ == '__main__':
    main()
