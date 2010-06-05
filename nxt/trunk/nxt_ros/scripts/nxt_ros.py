#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_ros')  
import nxt.locator
import rospy
import math
from nxt.motor import PORT_A, PORT_B, PORT_C
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
import nxt.sensor 
import nxt.motor 
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nxt_msgs.msg import Range

POWER_TO_NM = 0.02


def check_params(ns, params):
    for p in params:
        if not rospy.get_param(ns+'/'+p):
            return False
    return True


class Motor:
    def __init__(self, params, comm):
        # create motor
        self.name = params['name']
        self.motor = nxt.motor.Motor(comm, eval(params['port']))

        # create publisher
        self.pub = rospy.Publisher('joint_state', JointState)
        self.last_js = None
        
    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        state = self.motor.get_output_state()
        js.name.append(self.name)
        js.position.append(state[9] * math.pi / 180.0)
        js.effort.append(state[1] * POWER_TO_NM)
        vel = 0
        if self.last_js:
            vel = (js.position[0]-self.last_js.position[0])/(js.header.stamp-self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js




class Touch:
    def __init__(self, params, comm):
        # create touch sensor
        self.touch = nxt.sensor.TouchSensor(comm, eval(params['port']))

        # create publisher
        self.pub = rospy.Publisher(params['name'], Bool)
        
    def trigger(self):
        bl = Bool()
        bl.data = self.touch.get_sample()
        self.pub.publish(bl)



class UltraSonic:
    def __init__(self, params, comm):
        # create ultrasonic sensor
        self.touch = nxt.sensor.UltrasonicSensor(comm, eval(params['port']))

        # create publisher
        self.pub = rospy.Publisher(params['name'], Range)
        
    def trigger(self):
        ds = Range()
        ds.range = self.touch.get_sample()
        self.pub.publish(ds)





def main():
    sock = nxt.locator.find_one_brick()
    b = sock.connect()

    ns = 'nxt_ros'
    rospy.init_node(ns)
    config = rospy.get_param(ns)
    components = []
    for c in config:
        print c
        if c['type'] == 'motor':
            components.append(Motor(c, b))
        if c['type'] == 'touch':
            components.append(Touch(c, b))
        if c['type'] == 'ultrasonic':
            components.append(UltraSonic(c, b))


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        for c in components:
            c.trigger()
        rate.sleep()



if __name__ == '__main__':
    main()
