#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import csv
import os
import sys
from std_msgs.msg import String
import time
import urllib
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import Adafruit_SSD1306
import subprocess

class OLED_Display(Node):
    
    def __init__(self):
        super().__init__('oled_display')
        # Set constants, setup publishers and subscribe to topics.
        self.get_logger().info("Initializing OLED Display...")
        
        self.stats = {
            "MemoryUsage": "",
            "DiskUsage": ""
        }
        self.text_to_display = ""
        self.network_interfaces_to_track = ['eth0', 'wlan0']
        self.network_details = {
            "IPAddress": {}
        }
        #for interface in self.network_interfaces_to_track:
         #   self.network_details['IPAddress'][interface] = self.get_ip_address(interface)
            
        self.start_subscriptions()
        self.init_display()
        
    def start_subscriptions(self):
        self.create_subscription(String, '/stats/mem_usage', self.on_memory_usage, 10)
        self.create_subscription(String, '/stats/disk_usage', self.on_disk_usage, 10)
        self.create_subscription(String, '/oled/display', self.on_user_text, 1)
    
    def on_disk_usage(self, msg):
        self.stats['DiskUsage'] = msg.data

    def on_memory_usage(self, msg):
        self.stats['MemoryUsage'] = msg.data

    def init_display(self):
        
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1) # setting gpio to 1 is hack to avoid platform detection
        self.disp.begin()
        self.disp.clear()
        self.disp.display()
        
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.image = Image.new('1', (self.disp.width, self.disp.height))

        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.disp.width,self.disp.height), outline=0, fill=0)

        # Load default font.
        self.font = ImageFont.load_default()
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.loop_callback)
        
    def loop_callback(self):
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        top = padding
        bottom = self.disp.height-padding
        # Move left to right keeping track of the current x position for drawing shapes.
        x = 0
        
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.disp.width,self.disp.height), outline=0, fill=0)

        # Write two lines of text.
        if not self.text_to_display is None:
            self.draw.text((x, top), self.text_to_display,  font=self.font, fill=255)
        #else:
            #self.draw.text((x, top), "eth0: " + str(self.network_details['IPAddress']['eth0']), font=self.font, fill=255)

        #self.draw.text((x, top+8),  "wlan0: " + str(self.network_details['IPAddress']['wlan0']), font=self.font, fill=255)
        
        self.draw.text((x, top+16), str(self.stats['MemoryUsage']),  font=self.font, fill=255)
        self.draw.text((x, top+25), str(self.stats['DiskUsage']),  font=self.font, fill=255)

        self.disp.image(self.image)
        self.disp.display()
    
    def get_ip_address(self, interface):
                  
        if self.get_network_interface_state(interface) == 'down':
            return None
        cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % interface
                  
        return subprocess.check_output(cmd, shell=True).decode('ascii')[:-1]
    
    def get_network_interface_state(self, interface):              
        return subprocess.check_output('cat /sys/class/net/%s/operstate' % interface, shell=True).decode('ascii')[:-1]
    
    def on_user_text(self, msg):
        self.text_to_display = msg.data
           

def main(args=None):

    rclpy.init(args=args)
    
    node = OLED_Display()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
        
if __name__ == '__main__':
        main()