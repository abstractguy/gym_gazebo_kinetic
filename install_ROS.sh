sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
sudo apt update && \
sudo apt install -y ros-melodic-desktop-full && \
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
source ~/.bashrc && \
source /opt/ros/melodic/setup.bash && \
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential && \
echo 'KERNEL=="ttyUSB*", MODE="0666"' > /etc/udev/rules.d/ttyUSB.rules && \
pip2 install pyuarm --user && \
LINE_TO_ADD=$(lsusb | grep Arduino | cut -d" " -f6 | xargs -I{} echo "UARM_HWID_KEYWORD = \"USB VID:PID={}\"") sed -i "s|^UARM_HWID_KEYWORD.*$|$LINE_TO_ADD|g" /usr/local/lib/python2.7/dist-packages/pyuarm/tools/list_uarms.py && \
python2 -c 'import pyuarm.tools.firmware; pyuarm.tools.firmware' && \
cd ~/catkin_ws/src && \
git clone https://github.com/uArm-Developer/UArmForROS.git && \
cd ~/catkin_ws && \
catkin_make && \
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
source ~/.bashrc
