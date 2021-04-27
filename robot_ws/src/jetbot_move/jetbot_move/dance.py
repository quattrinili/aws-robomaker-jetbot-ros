#!/usr/bin/env python2

import rclpy
from rclpy.node import Node
import json
import csv
import os
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import urllib

# Name of this ROS package.
ROSAPP='jetbot_app'

# Topics to subscribe to
DANCE_ROUTINE_DEMO='/dance/demo'
DANCE_ROUTINE_JSON='/dance/raw'
DANCE_ROUTINE_START='/dance/start'


class Dance(Node):
    '''
    This class will move the JetBot robot according to a defined dance routine. 
    It subscribes to the following topics:

    /dance/demo -- Will pull the routine from /src/jetbot_app/routines folder in this ROS package.
    /dance/start -- The name of the routine stored in the public s3 bucket: s3://jetbot-dance-routines.
    /dance/raw -- A raw JSON string containing the routine.

    Examples:
    rostopic pub /dance/demo std_msgs/String { "routine": "autumn", "start_timestamp": "1607551421" } -- This command will use the routine: {ROS_PACKAGE_SHARE}/routines/autumn.json at timestamp 1607551421.
    rostopic pub /dance/start std_msgs/String { "routine": "autumn", "start_timestamp": "1607551421" } -- This command will use the routine: s3://{S3_BUCKET}/autumn.json at timestamp 1607551421.
    rostopic pub /dance/raw std_msgs/String "{ <DANCE_JSON> }" -- This command will use the raw JSON in the message and start the dance imminently. 

    Routine JSON is the following structure:
    {
        "name": "<DANCE_ROUTINE_NAME>",
        "songName": "<SONG_NAME>",
        "artist": "<ARTIST_NAME>",
        "audioURL": "<URL_TO_AUDIO_FILE>",
        "dancers": {
            "<ROBOT_DANCER_POSITION>": {
                "startPosition": "<START_ROBOT_POSITION>",
                "routine": {
                    "<TIME_STEP_TO_SEND_MOVE>": "<MOVE_COMMAND>",
                }
            }
        }
    }
    '''
    
    def __init__(self):

        self.declare_parameters(
            namespace='',
            parameters=[
                ('cmd_vel_topic', '/cmd_vel'),
                ('cmd_str_topic', '/cmd_str'),
                ('dance_start_demo', '/dance/demo'),
                ('dance_start_json', '/dance/raw'),
                ('dance_start_routine', '/dance/start'),
                ('s3_bucket', 'jetbot-dance-routines.s3-us-west-2.amazonaws.com'),
                ('speed', 3)
            ]
        )
        
        # Set constants, setup publishers and subscribe to topics.
        self.get_logger().info("Initializing dance settings...")
        self.get_logger().info("Command topics:")
        self.get_logger().info(" -- Velocity: %s" % cmd_vel_topic)
        self.get_logger().info(" -- String: %s" % cmd_str_topic)

        self.dancer_position = dancer_position
        self.twist = Twist()
        self.rate = self.create_rate(self.get_parameter('speed')._value)
        self.routine = []
        self.step_counter = 0
        self.routines_s3_bucket = self.get_parameter('s3_bucket')._value
        self.dancing = False
        self.start_timestamp = None
        
        self._cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic')._value, 1)
        self._cmd_str_pub = self.create_publisher(String, self.get_parameter('cmd_str_topic')._value, 1)        
        self.create_subscription(String, self.get_parameter('dance_routine_start_demo')._value, self.start_demo, 1)
        self.create_subscription(String, self.get_parameter('dance_routine_start_json')._value, self.start_json, 1)
        self.create_subscription(String, self.get_parameter('dance_routine_start')._value, self.start_routine, 1)
    
    def set_routine_array(self, routine):
        # Convert easy JSON document structure into more usable ordered array.
        self.routine = []
        for key, value in routine.items():
            self.routine.append({"step": int(key), "value": value})
        self.routine.sort()
        
    def start_demo(self, data):
        # Start a new demo dance based on local routines in /src/jetbot_app/routines.
        if (not self.dancing):
            meta = json.loads(data.data)
            self.get_logger().info('Dance demo message recieved. Looking up routine: %s' % meta['routine'])
            routines_path = "%s/routines/%s.json" % (self.rospack.get_path("jetbot_app"), meta['routine'])
            self.get_logger().info("Routines path: %s" % routines_path)
            if (os.path.exists(routines_path)):
                with open(routines_path) as f:
                    routine_json = json.load(f) 
                self.set_routine_array(routine_json['dancers'][self.dancer_position]['routine'])
            self.start_timestamp = int(meta['start_timestamp'])
            self.dancing = True
            self.dance(0)
        else:
            self.get_logger().info('Already dancing!')
            
    def start_routine(self, data):
        # Download a public dance routine and run it. 
        if (not self.dancing):
            meta = json.loads(data.data)
            url = "https://%s/%s.json" % (self.routines_s3_bucket, meta['routine'])
            self.get_logger().info('Downloading dance routine: %s' % url)
            response = urllib.urlopen(url)
            routine_json = json.loads(response.read())
            self.set_routine_array(routine_json['dancers'][self.dancer_position]['routine'])
            self.start_timestamp = int(meta['start_timestamp'])
            self.dancing = True
            self.dance(0)
        else:
            self.get_logger().info('Already dancing!')

    def start_json(self, data):
        # Recieve raw JSON from topic /dance/raw and start dance.
        if (not self.dancing):
            self.get_logger().info('JSON Routine recieved: %s' % data.data)
            routine_json = json.loads(data.data)
            self.set_routine_array(routine_json['dancers'][self.dancer_position]['routine'])
            self.start_timestamp = int(time.time())
            self.dancing = True
            self.dance(0)
        else:
            self.get_logger().info('Already dancing!')
            
    def move(self, value):
        # Publish move commands to the move node. If it is a Twist message, use cmd_vel.
        twist = value.split(" ")
        if len(twist) > 2:
            self.get_logger().info('Robot %s: New twist message %s.', self.dancer_position, value)
            self.twist.linear.x = float(twist[0])
            self.twist.linear.y = float(twist[1])
            self.twist.linear.z = float(twist[2])
            self.twist.angular.x = float(twist[3])
            self.twist.angular.y = float(twist[4])
            self.twist.angular.z = float(twist[5])
            self._cmd_vel_pub.publish(self.twist)
            return
        else:
            self.get_logger().info('Robot %s: New move %s.', self.dancer_position, value)
            self._cmd_str_pub.publish(value)
            return
          
    def dance(self, step): 
        # Recursive Dance Step Function, iterates by time step (controlled by SPEED in Hz), then matches current timestep to dance step in the routine.
        # Example: 
        # time step (20) : dance step [3] > { step: 20, value: "left" }, then move left and increment dance step
        # time step (21) : dance step [4] > { step: 23, value: "right"}, NO MATCH : NO CHANGE
        # time step (22) : dance step [4] > { step: 23, value: "right"}, NO MATCH : NO CHANGE
        # time step (23) : dance step [4] > { step: 23, value: "right"}, then move right and increment dance step
        # time step (24) : dance step [5] > { step: 32, value: "forward"}, NO MATCH : NO CHANGE
        next_move = "NO CHANGE"
        if (int(time.time())>=self.start_timestamp):
            if (self.routine[self.step_counter]['step'] == step):
                next_move = self.routine[self.step_counter]['value']
                if (next_move == 'end'):
                    self.dancing = False
                else:
                    self.move(next_move)
                self.step_counter += 1

            self.get_logger().info('Robot time step %i, dance step %i: %s.' % (step, self.step_counter, next_move))
            
            if (self.dancing and self.step_counter < len(self.routine)):
                self.rate.sleep()
                self.dance(step+1)
            else:
                self.get_logger().info('Dance finished!')
                self.move("stop")
                self.step_counter = 0
        else:
            time_left = int(self.start_timestamp-time.time())
            self.get_logger().info('Dance routine invoked. Starting in %i seconds' % time_left)
            self.rate.sleep()
            self.dance(step)
            

def main(args=None):

    rclpy.init(args=args)

    node = DanceNode()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
        
if __name__ == '__main__':
        main()