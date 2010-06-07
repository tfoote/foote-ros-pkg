#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_apps')  
import nxt.locator
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from nxt_msgs.msg import Range, JointCommand



class JointPositionController:
    def __init__(self):
        self.initialized = False
        self.vel = 0

        # get joint name
        self.name = rospy.get_param('name', 'motor_1')
        
        # joint interaction
        self.pub = rospy.Publisher('joint_command', JointCommand)
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # desired joint position
        rospy.Subscriber('joint_position', JointCommand, self.jnt_pos_cb)
        


    def jnt_pos_cb(self, msg):
        if msg.name == self.name:
            self.pos_desi = msg.effort


    def jnt_state_cb(self, msg):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name == self.name:
                self.vel = 0.5 * self.vel + 0.5 * vel
                if not self.initialized:
                    self.pos_desi = pos
                    self.initialized = True
                cmd = JointCommand()
                cmd.name = self.name
                cmd.effort = 190.0 * (self.pos_desi - pos) - 4.0 * self.vel
                print 'Joint at %f, going to %f, commanding joint %f'%(pos,self.pos_desi, cmd.effort)
                self.pub.publish(cmd)



def main():
    rospy.init_node('jnt_pos_controller')
    jnt_pos_controller = JointPositionController()
    rospy.spin()



if __name__ == '__main__':
    main()
