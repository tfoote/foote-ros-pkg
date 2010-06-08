#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_apps')  
import nxt.locator
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand


RADIUS = 2.0
K_ROT = 10.0
K_TRANS = 10.0

class JointPositionController:
    def __init__(self):
        self.initialized = False
        self.vel_rot_desi = 0
        self.vel_trans_desi = 0
        self.vel_trans = 0
        self.vel_rot = 0


        # get joint name
        self.l_joint = rospy.get_param('name', 'l_wheel')
        self.r_joint = rospy.get_param('name', 'r_wheel')
        
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
        self.vel_trans = 0.5*self.vel_trans + 0.5*(velocity[self.r_joint] + velocity[self.l_joint])/2.0
        self.vel_rot   = 0.5*self.vel_rot   + 0.5*(velocity[self.r_joint] - velocity[self.l_joint])/RADIUS
        print self.vel_rot

        
        l_cmd = JointCommand()
        l_cmd.name = self.l_joint
        l_cmd.effort = K_TRANS*(self.vel_trans_desi - self.vel_trans) - K_ROT*(self.vel_rot_desi - self.vel_rot)
        self.pub.publish(l_cmd)

        r_cmd = JointCommand()
        r_cmd.name = self.r_joint
        r_cmd.effort = K_TRANS*(self.vel_trans_desi - self.vel_trans) + K_ROT*(self.vel_rot_desi - self.vel_rot)
        self.pub.publish(r_cmd)

        print "commanding %f %f"%(l_cmd.effort, r_cmd.effort)


def main():
    rospy.init_node('jnt_pos_controller')
    jnt_pos_controller = JointPositionController()
    rospy.spin()



if __name__ == '__main__':
    main()
