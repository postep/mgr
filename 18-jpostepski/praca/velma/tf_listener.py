#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('velma_tf_listener')

    listener = tf.TransformListener()


    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        # try:
        listener.waitForTransform('/world', '/right_arm_cmd',rospy.Time(), rospy.Duration(400.0))
        (trans_cmd,rot_cmd) = listener.lookupTransform('/world', '/right_arm_cmd', rospy.Time())
        listener.waitForTransform('/world', '/right_arm_tool',rospy.Time(), rospy.Duration(4.0))
        now = rospy.Time.now()
        (trans_tool,rot_tool) = listener.lookupTransform('/world', '/right_arm_tool', rospy.Time())
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # continue

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)
        print trans_cmd, rot_cmd, trans_tool, rot_tool, now

        rate.sleep()
