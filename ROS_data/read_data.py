#! /usr/bin/env python
""" A script for reading rosbag data. """
import rosbag
if
__name__ == ’__main__’:
bag = rosbag.Bag(’2017-10-23-16-41-35.bag’)
for
(topic, msg, t)
in
bag.read_messages(topics=['/ardrone_velocity/pose_ref']):
print
(topic, msg, t)