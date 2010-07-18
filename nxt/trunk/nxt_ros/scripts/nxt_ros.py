#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_ros')  
import nxt.locator
import rospy
import math
from nxt.motor import PORT_A, PORT_B, PORT_C
from nxt.sensor import PORT_1, PORT_2, PORT_3, PORT_4
from nxt.sensor import Type
import nxt.sensor 
import nxt.motor 
import thread
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool
from nxt_msgs.msg import Range, Contact, JointCommand, Color, Gyro, Accelerometer
from PyKDL import Rotation

POWER_TO_NM = 0.01
POWER_MAX = 125

global my_lock
my_lock = thread.allocate_lock()

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
        self.cmd = 0 #default command

        # create publisher
        self.pub = rospy.Publisher('joint_state', JointState)
        self.last_js = None
        
        # create subscriber
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)


    def cmd_cb(self, msg):
        if msg.name == self.name:
 
            cmd = msg.effort / POWER_TO_NM
            if cmd > POWER_MAX:
                cmd = POWER_MAX
            elif cmd < -POWER_MAX:
                cmd = -POWER_MAX
            self.cmd = cmd  #save command

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
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        # send command
        self.motor.run(int(self.cmd), 0)




class TouchSensor:
    def __init__(self, params, comm):
        # create touch sensor
        self.touch = nxt.sensor.TouchSensor(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Contact)
        
    def trigger(self):
        ct = Contact()
        ct.contact = self.touch.get_sample()
        ct.header.frame_id = self.frame_id
        ct.header.stamp = rospy.Time.now()
        self.pub.publish(ct)



class UltraSonicSensor:
    def __init__(self, params, comm):
        # create ultrasonic sensor
        self.ultrasonic = nxt.sensor.UltrasonicSensor(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Range)
        
    def trigger(self):
        ds = Range()
        ds.header.frame_id = self.frame_id
        ds.header.stamp = rospy.Time.now()
        ds.range = self.ultrasonic.get_sample()/100.0
        ds.spread_angle = 0.04
        ds.range_min = 0.0
        ds.range_max = 2.54
        self.pub.publish(ds)

class GyroSensor:
    def __init__(self, params, comm):
        #create gyro sensor
        self.gyro = nxt.sensor.GyroSensor(comm, eval(params['port']))
        self.frame_id = params['frame_id']
        self.orientation = 0.0
        self.offset = 0.0
        self.prev_time = rospy.Time.now()

        # calibrate
        rospy.loginfo("Calibrating Gyro. Don't move the robot now")
        start_time = rospy.Time.now()
        cal_duration = rospy.Duration(2.0)
        offset = 0
        tmp_time = rospy.Time.now()
        while rospy.Time.now() < start_time + cal_duration:
            rospy.sleep(0.01)
            sample = self.gyro.get_sample()
            now = rospy.Time.now()
            offset += (sample * (now - tmp_time).to_sec())
            tmp_time = now
        self.offset = offset / (tmp_time - start_time).to_sec()
        rospy.loginfo("Gyro calibrated with offset %f"%self.offset)

        # create publisher
        self.pub = rospy.Publisher(params['name'], Gyro)

        # create publisher
        self.pub2 = rospy.Publisher(params['name']+"_imu", Imu)

    def trigger(self):
        sample = self.gyro.get_sample()
        gs = Gyro()
        gs.header.frame_id = self.frame_id
        gs.header.stamp = rospy.Time.now()
        gs.calibration_offset.x = 0.0
        gs.calibration_offset.y = 0.0
        gs.calibration_offset.z = self.offset
        gs.angular_velocity.x = 0.0
        gs.angular_velocity.y = 0.0
        gs.angular_velocity.z = (sample-self.offset)*math.pi/180.0
        gs.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
        self.pub.publish(gs)

        imu = Imu()
        imu.header.frame_id = self.frame_id
        imu.header.stamp = rospy.Time.now()
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = (sample-self.offset)*math.pi/180.0
        imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
        imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]
        self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
        self.prev_time = imu.header.stamp
        (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = Rotation.RotZ(self.orientation).GetQuaternion()
        self.pub2.publish(imu)

class AccelerometerSensor:
    def __init__(self, params, comm):
        #create gyro sensor
        self.accel = nxt.sensor.AccelerometerSensor(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Accelerometer)

    def trigger(self):
        gs = Accelerometer()
        gs.header.frame_id = self.frame_id
        gs.header.stamp = rospy.Time.now()
        x,y,z = self.accel.get_sample()
        gs.linear_acceleration.x = x*9.8
        gs.linear_acceleration.y = y*9.8
        gs.linear_acceleration.z = z*9.8
        gs.linear_acceleration_covariance = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.pub.publish(gs)

class ColorSensor:
    def __init__(self, params, comm):
        # create color sensor
        self.color = nxt.sensor.ColorSensor(comm, eval(params['port']))
        self.frame_id = params['frame_id']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Color)
        
    def trigger(self):
        co = Color()
        co.header.frame_id = self.frame_id
        co.header.stamp = rospy.Time.now()
        co.intensity = 0.0
        color = self.color.get_color()
        if color == 1:  # black
            co.r = 0.0
            co.g = 0.0
            co.b = 0.0
        elif color == 2: # blue
            co.r = 0.0
            co.g = 0.0
            co.b = 1.0
        elif color == 3: # green
            co.r = 0.0
            co.g = 1.0
            co.b = 0.0
        elif color == 4: # yellow
            co.r = 1.0
            co.g = 1.0
            co.b = 0.0
        elif color == 5: # red
            co.r = 1.0
            co.g = 0.0
            co.b = 1.0
        elif color == 6: # white
            co.r = 1.0
            co.g = 1.0
            co.b = 1.0
        else:
            rospy.logerr('Undefined color of color sensor')
        self.pub.publish(co)


class IntensitySensor:
    def __init__(self, params, comm):
        # create intensity sensor
        self.intensity = nxt.sensor.ColorSensor(comm, eval(params['port']))
        self.frame_id = params['frame_id']
        self.color_r = params['color_r']
        self.color_g = params['color_g']
        self.color_b = params['color_b']

        if self.color_r == 1.0 and self.color_g == 0.0 and self.color_b == 0.0:
            self.color = 'red'
        elif self.color_r == 0.0 and self.color_g == 1.0 and self.color_b == 0.0:
            self.color = 'green'
        elif self.color_r == 0.0 and self.color_g == 0.0 and self.color_b == 1.0:
            self.color = 'blue'
        elif self.color_r == 0.0 and self.color_g == 0.0 and self.color_b == 0.0:
            self.color = 'off'
        else:
            rospy.logerr('Invalid RGB values specifies for intensity color sensor')

        # create publisher
        self.pub = rospy.Publisher(params['name'], Color)
        
    def trigger(self):
        co = Color()
        co.header.frame_id = self.frame_id
        co.header.stamp = rospy.Time.now()
        co.r = self.color_r
        co.g = self.color_g
        co.b = self.color_b
        co.intensity = self.intensity.get_reflected_light(self.color)
        self.pub.publish(co)

def main():
    ns = 'nxt_robot'
    rospy.init_node('nxt_ros')
    host = rospy.get_param("~host", None)
    sock = nxt.locator.find_one_brick(host)
    b = sock.connect()

    config = rospy.get_param("~"+ns)
    components = []
    for c in config:
        print c
        if c['type'] == 'motor':
            components.append(Motor(c, b))
        elif c['type'] == 'touch':
            components.append(TouchSensor(c, b))
        elif c['type'] == 'ultrasonic':
            components.append(UltraSonicSensor(c, b))
        elif c['type'] == 'color':
            components.append(ColorSensor(c, b))
        elif c['type'] == 'intensity':
            components.append(IntensitySensor(c, b))
        elif c['type'] == 'gyro':
            components.append(GyroSensor(c, b))
        elif c['type'] == 'accelerometer':
            components.append(AccelerometerSensor(c, b))
        else:
            rospy.logerr('Invalid sensor/actuator type %s'%c['type'])

    while not rospy.is_shutdown():
        my_lock.acquire()
        for c in components:
            c.trigger()
        my_lock.release()
        rospy.sleep(0.1)



if __name__ == '__main__':
    main()
