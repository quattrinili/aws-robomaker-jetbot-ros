#!/bin/bash
if [ ! -f /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth ]; then
    mkdir -p /home/ggc_user/.local/lib/python3.6/site-packages/
    touch /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/lib/python3.6" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages/jetbot-0.3.0-py3.6.egg" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages/Adafruit_MotorHAT-1.4.0-py3.6.egg" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages/Adafruit_SSD1306-1.6.2-py3.6.egg" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages/Adafruit_GPIO-1.0.3-py3.6.egg" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages/spidev-3.4-py3.6-linux-aarch64.egg" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/local/lib/python3.6/dist-packages/Adafruit_PureIO-0.2.3-py3.6.egg" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/lib/python3/dist-packages" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
    echo "/usr/lib/python3.6/dist-packages" >> /home/ggc_user/.local/lib/python3.6/site-packages/py3.pth
fi
ROS_SHARE_PATH=$(rospack find jetbot_app)
cd $ROS_SHARE_PATH
cd ../../lib/jetbot_inference
python3 run.py