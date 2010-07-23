#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_apps')  
import nxt.locator
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand


WHEEL_RADIUS = 0.044/2.0
WHEEL_BASIS  = 0.11/2.0
VEL_TO_EFF = 0.5
K_ROT = 0.075/VEL_TO_EFF
K_TRANS = 0.055/VEL_TO_EFF


class BaseController:
    def __init__(self):
        self.initialized = False
        self.vel_rot_desi = 0
        self.vel_trans_desi = 0
        self.vel_trans = 0
        self.vel_rot = 0


        # get joint name
        self.l_joint = rospy.get_param('l_wheel_joint', 'l_wheel_joint')
        self.r_joint = rospy.get_param('r_wheel_joint', 'r_wheel_joint')
        
        # joint interaction
        self.pub = rospy.Publisher('joint_command', JointCommand)
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # base commands
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        


    def cmd_vel_cb(self, msg):
        self.vel_rot_desi = msg.angular.z
        self.vel_trans_desi = msg.linear.x


    def jnt_state_cb(self, msg):
        velocity = {}
        for name, vel in zip(msg.name, msg.velocity):
            velocity[name] = vel

        # lowpass for measured velocity
        self.vel_trans = 0.5*self.vel_trans + 0.5*(velocity[self.r_joint] + velocity[self.l_joint])*WHEEL_RADIUS/2.0
        self.vel_rot =   0.5*self.vel_rot   + 0.5*(velocity[self.r_joint] - velocity[self.l_joint])*WHEEL_RADIUS/(2.0*WHEEL_BASIS)

        # velocity commands
        vel_trans = self.vel_trans_desi + K_TRANS*(self.vel_trans_desi - self.vel_trans)
        vel_rot = self.vel_rot_desi + K_ROT*(self.vel_rot_desi - self.vel_rot)
        
        # wheel commands
        l_cmd = JointCommand()
        l_cmd.name = self.l_joint
        l_cmd.effort = VEL_TO_EFF*(vel_trans/WHEEL_RADIUS - vel_rot*WHEEL_BASIS/WHEEL_RADIUS)
        self.pub.publish(l_cmd)

        r_cmd = JointCommand()
        r_cmd.name = self.r_joint
        r_cmd.effort = VEL_TO_EFF*(vel_trans/WHEEL_RADIUS + vel_rot*WHEEL_BASIS/WHEEL_RADIUS)
        self.pub.publish(r_cmd)


def main():
    rospy.init_node('base_controller')
    base_controller = BaseController()
    rospy.spin()



if __name__ == '__main__':
    main()
