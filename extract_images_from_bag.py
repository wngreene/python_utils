#!/usr/bin/env python

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        timestamp = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        cv2.imwrite(os.path.join(args.output_dir, "%.6f.png" % (timestamp.to_sec())), cv_img)
        print "Wrote image %i" % count

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
