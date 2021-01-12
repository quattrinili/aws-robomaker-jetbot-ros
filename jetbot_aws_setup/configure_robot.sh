#!/bin/sh

# This script must be copied to the Jetbot and run from a shell terminal with sudo privileges. 
#
# The script will download the install the latest ARM64 version of Greengrass onto the robot,
# configure the robot with Greengrass certificates and start the Greengrass daemon. 
#

# greengrass version to install
ggc_version=1.11.0

# greengrass certs bundle downloaded from RoboMaker console
# supplied as an argument to this script
certs=$1

if [ "$#" -ne 1 ]
then
  echo "Usage: configure_robot.sh </path/to/certs_bundle.zip>"
  exit 1
fi

if [ ! -f "$certs" ]
then
  echo "Unable to open file: $certs"
  exit 1
fi

# download Greengrass Core software
cd /home/jetbot && wget -nd https://d1onfpft10uf5o.cloudfront.net/greengrass-core/downloads/$ggc_version/greengrass-linux-aarch64-$ggc_version.tar.gz

# extract Greengrass Core software into /greengrass folder on the robot
sudo tar -xzvf greengrass-linux-aarch64-$ggc_version.tar.gz -C /

# extract robot's Greengrass certificates to /greengrass folder
sudo unzip -o $certs -d /greengrass

# update the CA certificate used by RoboMaker
cd /greengrass/certs/ && sudo wget -O root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

# create ggc_user and group
sudo adduser --system ggc_user
sudo addgroup --system ggc_group

# set /greengrass directory ownership to ggc_user
sudo chown -R ggc_user:ggc_group /greengrass

# add ggc_user to i2c group
sudo usermod -a -G i2c ggc_user

# add a systemd startup script
sudo cat >>/etc/systemd/system/greengrass.service << EOF
[Unit]
Description=Greengrass Daemon

[Service]
Type=forking
PIDFile=/var/run/greengrassd.pid
Restart=on-failure
ExecStart=/greengrass/ggc/core/greengrassd start
ExecReload=/greengrass/ggc/core/greengrassd restart
ExecStop=/greengrass/ggc/core/greengrassd stop

[Install]
WantedBy=multi-user.target
EOF

# enable systemd service and start Greengrass Core
sudo systemctl enable greengrass.service
sudo systemctl start greengrass.service

echo "Done setting up the Jetbot!"
