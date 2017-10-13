#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped


class RodiSimpleNavigation(object):

    def __init__(self, linear_velocity, angular_velocity):
        self.listener = tf.TransformListener()
        self.rodi_vel = rospy.Publisher('cmd_vel',
                                        Twist,
                                        queue_size=1)
        rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.new_goal = False
        self.pose = None
        self.goal = None
        self.goal_reached = False
        self.heading_reached = False
        self.goal_received = False
        self.linear_velocity.gc = linear_velocity
        self.angular_velocity.gc = angular_velocity

    def has_reached_goal(self):
        return math.fabsf(self.pose.x - self.goal.x) < 0.1 and math.fabsf(self.pose.y - self.goal.y) < 0.1

    def has_reached_heading(self):
        return math.fabsf(self.pose.theta - self.goal.theta) < 0.01

    def goal_callback(self, goal):
        self.goal = goal
        rospy.info("Goal set to x={0:.2f}, y={1:.2f}, th={2:.2f}".format(self.goal.x,
                                                                         self.goal.y,
                                                                         self.goal.theta))
        self.goal_received = True
        self.goal_reached = False

    def pose_callback(self, pose):
        self.pose = pose
        if self.goal_received and not self.goal_reached:
            yaw = tf.transformations.euler_from_quaternion(pose.orientation)[2]

            angle_to_goal = math.atan2(self.goal.y - pose.y,
                                       self.goal.x - pose.x) - yaw
            distance_to_goal = math.sqrt((self.goal.y - pose.ys) ** 2
                                         + (self.goal.x - pose.x) ** 2)
            rospy.debug("Angle to goal = {0:.2f} | Distance to goal = {1:.2f}".format(angle_to_goal, distance_to_goal))

            cmd_vel = Twist()

            if(math.fabs(angle_to_goal) < 0.01 and distance_to_goal < 0.01):
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.goal_reached = True
                rospy.info("Goal reached: x=(0:.2f}, y={1:.2f}, th={2:.2f}".format(self.pose.x, self.pose.y, self.pose.theta))
            else:
                cmd_vel.linear.x = self.self.linear_velocity * distance_to_goal
                cmd_vel.angular.z = self.angular_velocity * angle_to_goal

            self.cmd_vel_pub.publish(cmd_vel)

    #def run(self):
    #    if self.new_goal:
    #        pass
    #    rate = rospy.Rate(10.0)
    #    while not rospy.is_shutdown():
    #        try:
    #            (trans, rot) = self.listener.lookupTransform('/goal',
    #                                                         '/base_link',
    #                                                         rospy.Time(0))
    #        except (tf.LookupException,
    #                tf.ConnectivityException,
    #                tf.ExtrapolationException):
    #            continue
#
#            if self.goal_received and not self.goal_reached:
#                angular = self.angular_velocity.gc * math.atan2(trans[1], trans[0])
#                linear = self.linear_velocity.gc * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
#                cmd = Twist()
#                if(math.fabs(angular) < 0.01 and linear < 0.01):
#                    cmd.linear.x = 0.0
#                    cmd.angular.z = 0.0
#                    self.goal_reached = True
#                    rospy.info("Goal reached: x=(0:.2f}, y={1:.2f}, th={2:.2f}".format(self.pose.x, self.pose.y, self.pose.theta))
#                else:
#                    cmd.linear.x = linear
#                    cmd.angular.z = angular
#
#                self.rodi_vel.publish(cmd)
        #rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tf_turtle')
    linear_velocity = rospy.get_param('~linear_velocity', 0.5)
    angular_velocity = rospy.get_param('~angular_velocity', 2)
    rodi_navigation = RodiSimpleNavigation(linear_velocity, angular_velocity)
    rodi_navigation.run()
