#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('rodi_pose')

    listener = tf.TransformListener()

    rodi_pose = rospy.Publisher('rodi/pose', geometry_msgs.msg.PoseStamped, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        yaw = tf.transformations.euler_from_quaternion(rot)
        pose = geometry_msgs.msg.PoseStamped()
        pose.position = trans
        pose.orientation = rot
        rodi_pose.publish(pose)

        rate.sleep()
