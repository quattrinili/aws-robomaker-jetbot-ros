## AWS-Enabled Waveshare Jetbot with NVidia Jetson Nano

### Hardware Getting Started Guide
**The following assumes the robot has been assembled according to Waveshare instructions (TODO: add assembly guide)**

#### First boot 
1. Download and flash SD card with the 2GB Jetson Nano image from Nvidia: https://jetbot.org/master/software_setup/sd_card.html
2. Plug SD card in, connect keyboard, HDMI and power and power on
3. Login to console (default username/password is **jetbot**/**jetbot**)
4. Configure Jetbot to connect to your Wifi network:
    ```
    $ sudo nmcli device wifi connect <SSID> password <PASSWORD>
    ```
5. Make a note of your Jetbot's IP address from OLED display

#### Install dependencies and configure Jetson
1. SSH into your Jetbot with IP address from above and install the following packages:
    ```
    $ sudo apt-get install python3-pip python3-setuptools rsync
    ```
2. Clone the Waveshare repository, it contains scripts customized for the Waveshare Jetbot:
    ```
    $ cd ~/
    $ git clone https://github.com/waveshare/jetbot waveshare_jetbot
    $ cd waveshare_jetbot
    $ sudo python3 setup.py install
    $ rsync -r ~/waveshare_jetbot/notebooks ~/Notebooks
    ```
3. Configure Jetson Nano power mode
    ```
    $ sudo nvpmodel -m1
    ```
4. Confirm power mode settings
    ```
    $ sudo nvpmodel -q
    ```
5. Build and install custom Waveshare Jetbot stats service (shows battery voltage on OLED display):
    ```
    $ cd ~/waveshare_jetbot/jetbot/jetbot/utils
    $ python3 create_stats_service.py
    $ sudo mv jetbot_stats.service /etc/systemd/system/jetbot_stats.service
    $ sudo systemctl enable jetbot_stats
    $ sudo systemctl start jetbot_stats

    ### the following dependencies are needed if running the systemd service above
    $ sudo apt-get install libffi6 libffi-dev libjpeg-dev
    $ sudo pip3 install traitlets pillow ipywidgets
    
    ### Install the pre-built PyTorch pip wheel for Jetpack 4.4
    ### Box link from forum post here: https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-7-0-now-available/72048/555
    ### check your Jetpack version with: sudo apt-cache show nvidia-jetpack
    $ sudo apt-get install libopenblas-base libopenmpi-dev python3-smbus
    $ wget https://nvidia.box.com/shared/static/wa34qwrwtk9njtyarwt5nvo6imenfy26.whl -O torch-1.7.0-cp36-cp36m-linux_aarch64.whl
    $ sudo pip3 install numpy torch-1.7.0-cp36-cp36m-linux_aarch64.whl
    # stop the Nvidia jetbot docker containers (this will stop the Jupyter service as well), reboot and confirmed the OLED display shows battery voltage now
    $ cd ~/jetbot/docker/
    $ ./disable.sh
    $ sudo reboot
    ```
