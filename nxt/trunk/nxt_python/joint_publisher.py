#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_python')                                  
import nxt.locator
import rospy
import math
from nxt.sensor import *
from nxt.motor import *
from sensor_msgs.msg import JointState

POWER_TO_NM = 0.02

class Motors:
    def __init__(self):
        self.sock = nxt.locator.find_one_brick()
        self.motors = []
        if self.sock:
            b = self.sock.connect()
            self.motors.append(Motor(b, PORT_A))
            self.motors.append(Motor(b, PORT_B))
            self.motors.append(Motor(b, PORT_C))
        else:
            print 'No NXT bricks found'
        self.sub = rospy.Subscriber('joint_commands', JointState, self.cmd_cb)


    def cmd_cb(self):
        print 'test'
        
    def spin(self):
        pub = rospy.Publisher('joint_states', JointState)
        last_js = None
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            js = JointState()
            js.header.stamp = rospy.Time.now()
            for m in self.motors:
                state = m.get_output_state()
                js.name.append('Motor_%d'%state[0])
                js.position.append(state[9] * math.pi / 180.0)
                js.effort.append(state[1] * POWER_TO_NM)
                index = len(js.position)-1
                vel = 0
                if last_js:
                    vel = (js.position[index]-last_js.position[index])/(js.header.stamp-last_js.header.stamp).to_sec()
                js.velocity.append(vel)
            pub.publish(js)
            last_js = js
            rate.sleep()


rospy.init_node('motor_state_pub')
m = Motors()
m.spin()
