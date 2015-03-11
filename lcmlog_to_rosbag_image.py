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

    roi_parser = parser.add_argument_group("Format7/ROI", "Format7/ROI options needed when dealing with non-standard video modes.")
    roi_parser.add_argument('--binning_x', default=1, type=int, dest='binning_x', help='Image binning factor.')
    roi_parser.add_argument('--binning_y', default=1, type=int, dest='binning_y', help='Image binning factor.')
    roi_parser.add_argument('--x_offset', default=0, type=int, dest='x_offset', help="ROI x offset (in UNBINNED pixels)")
    roi_parser.add_argument('--y_offset', default=0, type=int, dest='y_offset', help="ROI y offset (in UNBINNED pixels)")
    roi_parser.add_argument('--width', default=640, type=int, dest='width', help="ROI width (in UNBINNED pixels)")
    roi_parser.add_argument('--height', default=480, type=int, dest='height', help="ROI height (in UNBINNED pixels)")
    roi_parser.add_argument('--do_rectify', default=False, type=bool, dest='do_rectify', help="Do rectification when querying ROI.")
    args = parser.parse_args()

    if args.lcm_channels is None:
        args.lcm_channels = []
    if args.yml_files is None:
        args.yml_files = []

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
                    ros_msg.header.frame_id = "camera"

                    ros_msg.format = 'jpeg'

                    ros_msg.data = lcm_msg.data

                    # Fill in camera info
                    camera_info = CameraInfo()
                    camera_info.header = ros_msg.header
                    camera_info.height = y['image_height']
                    camera_info.width = y['image_width']

                    if y["distortion_model"] != "plumb_bob":
                        print "Encountered non-supported distorion model %s. Skipping..." % y["distortion_model"]
                        continue

                    camera_info.distortion_model = y["distortion_model"]
                    camera_info.D = y["distortion_coefficients"]['data']
                    camera_info.K = y["camera_matrix"]['data']
                    camera_info.R = y["rectification_matrix"]['data']
                    camera_info.P = y["projection_matrix"]['data']
                    camera_info.binning_x = args.binning_x
                    camera_info.binning_y = args.binning_y
                    camera_info.roi.x_offset = args.x_offset
                    camera_info.roi.y_offset = args.y_offset
                    camera_info.roi.height = args.height
                    camera_info.roi.width = args.width
                    camera_info.roi.do_rectify = args.do_rectify

                    bag.write("/camera/" + l + "/image_raw/compressed", ros_msg, ros_msg.header.stamp)
                    bag.write("/camera/" + l + "/camera_info", camera_info, camera_info.header.stamp)

                    count += 1

                    if count % 100 == 0:
                        print "Wrote %i events" % count
    finally:
        log.close()
        bag.close()

    print("Done.")

if __name__ == '__main__':
    main()
