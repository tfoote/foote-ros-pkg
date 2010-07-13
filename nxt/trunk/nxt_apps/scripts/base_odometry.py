#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_apps')  
import nxt.locator
import rospy
import math
import thread
import tf
from PyKDL import *
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nxt_msgs.msg import Range, JointCommand
from tf_conversions import posemath


WHEEL_RADIUS = 0.044/2.0
WHEEL_BASIS  = 0.11/2.0

class BaseOdometry:
    def __init__(self):
        self.initialized = False

        # get joint name
        self.l_joint = rospy.get_param('l_wheel_joint', 'l_wheel_joint')
        self.r_joint = rospy.get_param('r_wheel_joint', 'r_wheel_joint')

        # joint interaction
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # tf broadcaster
        self.br = tf.TransformBroadcaster()

        # publish results on topic
        self.pub = rospy.Publisher('odom', Odometry)

        self.initialized = False

    def jnt_state_cb(self, msg):
        # crates map
        position = {}
        for name, pos in zip(msg.name, msg.position):
            position[name] = pos
        
        # initialize
        if not self.initialized:
            self.r_pos = position[self.r_joint]
            self.l_pos = position[self.l_joint]
            self.pose = Frame()
            self.initialized = True
        else:
            delta_r_pos = position[self.r_joint] - self.r_pos
            delta_l_pos = position[self.l_joint] - self.l_pos
            delta_trans = (delta_r_pos + delta_l_pos)*WHEEL_RADIUS/2.0
            delta_rot   = (delta_r_pos - delta_l_pos)*WHEEL_RADIUS/(2.0*WHEEL_BASIS)
            twist = Twist(Vector(delta_trans, 0, 0),  Vector(0, 0, delta_rot))
            self.r_pos = position[self.r_joint]
            self.l_pos = position[self.l_joint]
            self.pose = addDelta(self.pose, self.pose.M * twist)
            self.br.sendTransform(self.pose.p, self.pose.M.GetQuaternion(), rospy.Time.now(), 'base_link', 'odom')

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose = posemath.toMsg(self.pose)
            odom.pose.covariance = [0.00001, 0, 0, 0, 0, 0,
                                    0, 0.00001, 0, 0, 0, 0, 
                                    0, 0, 10.0000, 0, 0, 0,
                                    0, 0, 0, 1.00000, 0, 0,
                                    0, 0, 0, 0, 1.00000, 0,
                                    0, 0, 0, 0, 0, 1.00000]            
            self.pub.publish(odom)

def main():
    rospy.init_node('base_odometry')
    base_odometry = BaseOdometry()
    rospy.spin()



if __name__ == '__main__':
    main()
