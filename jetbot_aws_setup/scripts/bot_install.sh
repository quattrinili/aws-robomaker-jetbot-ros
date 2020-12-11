
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ROS Base
sudo apt-get update
sudo apt-get install ros-melodic-ros-base
sudo rosdep init

# add ROS paths to environment
sudo sh -c 'echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc'
sudo usermod -aG i2c $USER
sudo pip3 install rospkg

sudo useradd -system ggc_user -home /home/ggc_user -shell /bin/bash
sudo groupadd -system ggc_group
cd /
sudo wget https://d1onfpft10uf5o.cloudfront.net/greengrass-core/downloads/1.10.0/greengrass-linux-aarch64-1.10.0.tar.gz
sudo tar -xvf greengrass-linux-aarch64-1.10.0.tar.gz
