#!/usr/bin/env python
"""
Spoofs a rigid body. Useful for testing the snap-stack autopilot offline.
"""

import argparse

import rospy

from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Spoof a rigid body as if it were published by mocap')
  parser.add_argument('--rigid-body', '-r', type=str, required=True)
  args = parser.parse_args()

  rospy.init_node('{}_spoofer'.format(args.rigid_body))
  pub = rospy.Publisher('/{}/world'.format(args.rigid_body), PoseStamped, queue_size=1)

  # build message with valid pose
  msg = PoseStamped()
  msg.header.frame_id = 'world'
  msg.pose.orientation.w = 1
  msg.pose.position.x = 1 # set away from origin so we know data is coming through

  rate = rospy.Rate(100) # Hz
  while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()