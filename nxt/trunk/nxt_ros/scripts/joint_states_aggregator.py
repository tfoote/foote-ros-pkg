#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_ros')  
import rospy
import math
from sensor_msgs.msg import JointState


class JS:
    def __init__(self, name, header, position, velocity, effort):
        self.name = name
        self.header = header
        self.position = position
        self.velocity = velocity 
        self.effort = effort


class JSAggregator:
    def __init__(self):
        # create motor
        self.sub = rospy.Subscriber('joint_state', JointState, self.callback)
        # create publisher
        self.pub = rospy.Publisher('joint_states', JointState)
        self.observed_states = {}
                 
        
    def callback(self, data):
        for i in xrange(0, len(data.name)):
            self.observed_states[data.name[i]] = JS(data.name[i], 
                                                    data.header, 
                                                    data.position[i], 
                                                    data.velocity[i], 
                                                    data.effort[i])
            
        todelete = [v for k, v in self.observed_states.iteritems() if  data.header.stamp - v.header.stamp > rospy.Duration().from_sec(1.0) ] #hack parametersize
        for td in todelete:
            del self.observed_states[td]


        msg_out = JointState()
        msg_out.header = data.header        
        for k, v in self.observed_states.iteritems():
            #print k, v
            msg_out.name.append(v.name)
            msg_out.position.append(v.position)
            msg_out.velocity.append(v.velocity)
            msg_out.effort.append(v.effort)
            
        self.pub.publish(msg_out)

def main():
    rospy.init_node("joint_state_aggregator")

    agg = JSAggregator()

    rospy.spin()



if __name__ == '__main__':
    main()
