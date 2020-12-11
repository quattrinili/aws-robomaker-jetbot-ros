#!/usr/bin/env python2
import rospy
import time
import json

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdConverter():
    
    def __init__(self, twist_speed, prefix):
        # Set constants, setup publishers and subscribe to topics.
        rospy.loginfo("Initializing converter...")
        self.twist_speed = twist_speed
        self.setTwist(0, 0)
        cmd_vel_topic = '/%s/cmd_vel' % prefix
        cmd_str_topic = '/%s/cmd_str' % prefix
        rospy.loginfo("Subscribing to the following topics...")
        rospy.loginfo(cmd_vel_topic)
        rospy.loginfo(cmd_str_topic)
        rospy.Subscriber(cmd_str_topic, String, self.on_cmd_str)
        rospy.Subscriber(cmd_vel_topic, Twist, self.on_cmd_vel)
        self.cmd_vel_pub = rospy.Publisher('jetbot_diff_controller/cmd_vel', Twist, queue_size=1)
    
    def setTwist(self, left_right, forward_back):
        twist = Twist()
        twist.angular.z = (left_right*5)
        twist.linear.x = forward_back
        self.twist_message = twist
        
    def getTwist(self):
        return self.twist_message
    
    def publishMessage(self):
        rospy.loginfo("Publishing message to %s" % str(self.twist_message))
        self.cmd_vel_pub.publish(self.twist_message)
        
    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_str(self, msg):
        self.convert(msg.data)
        
    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_vel(self, msg):
        self.twist_message = msg
        
    def convert(self, val):
        if val == "left":
            self.setTwist(-(self.twist_speed), 0)
        elif val == "right":
            self.setTwist(self.twist_speed, 0)
        elif val == "backward":
            self.setTwist(0, -(self.twist_speed))
        elif val == "forward":
            self.setTwist(0, self.twist_speed)
        elif val == "stop" or val == "end":
            self.setTwist(0, 0)
        
        rospy.loginfo("Converted string message: %s" % str(self.twist_message))

# initialization
if __name__ == '__main__':
    
    twist_speed = rospy.get_param("/twist_speed", 0.2)
    prefix = rospy.get_param("/cvt_prefix", "move")
    rospy.init_node('cmd_converter')
    cmd_converter = CmdConverter(float(twist_speed), prefix)
   
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_converter.publishMessage()
        r.sleep()
