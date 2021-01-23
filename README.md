# Overview of gym-gazebo
An OpenAI gym extension for using Gazebo known as `gym-gazebo`! This work can put gym environment with gazebo, then you would like putting robot into gazebo with code applying gym. You can also visit the official github here [gym-gazebo](https://github.com/erlerobot/gym-gazebo). If you use ROS2, the better way for you is visiting the newest version [gym-gazebo2](https://github.com/AcutronicRobotics/gym-gazebo2).

## Summary
Because the official github which install version to Ubuntu16.04 has been deprecated, and the package in author's github has many question that has been closed without issues, Here I provide installation of mine and some hint to problem solving.

The original Installation of authors is here [original](https://github.com/zhaolongkzz/gym_gazebo_kinetic/blob/kinetic/Introduction.md) and here [INSTALL](https://github.com/zhaolongkzz/gym_gazebo_kinetic/blob/kinetic/INSTALL.md). If this installation wouldn't help you, visiting the author's github and submit a issue.

## Prerequisites
- ubuntu16.04
- ROS-Kinetic
  &ensp;&ensp;(visit the official web [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).)
- Gazebo 7.14
- openai-gym
  &ensp;&ensp;(visit gym github [here](https://github.com/openai/gym.git).)
- anaconda3
  &ensp;&ensp;(install anaconda, click [here](http://docs.anaconda.com/anaconda/install/linux/).)
- python=2.7
  &ensp;&ensp;(with anaconda env below.)

If you want to train it with GPU here, you should install cuda
- cuda=9.0
- libcudnn7.3


## Installation of conda env

Create an environment to run them.
```bash
yes | conda create -n gymenv python=2.7 pip=19.3.1 numpy=1.16.2 matplotlib=2.2.3 protobuf=3.5.2
conda activate gymenv
yes | pip install gym rospkg catkin_pkg defusedxml netifaces
yes | pip install --upgrade scikit-image==0.14.2
git clone https://github.com/abstractguy/gym_gazebo_kinetic.git
cd gym_gazebo_kinetic
pip install -e .
```

## Installation of ROS
First installing some ROS dependencies below:
```bash
sudo apt update
sudo apt-get -y install cmake \
                        gcc \
                        g++ \
                        qt4-qmake \
                        libqt4-dev \
                        libusb-dev \
                        libftdi-dev \
                        ros-kinetic-octomap-msgs \
                        ros-kinetic-joy \
                        ros-kinetic-geodesy \
                        ros-kinetic-octomap-ros \
                        ros-kinetic-control-toolbox \
                        ros-kinetic-pluginlib \
                        ros-kinetic-trajectory-msgs \
                        ros-kinetic-control-msgs \
                        ros-kinetic-std-srvs \
                        ros-kinetic-nodelet \
                        ros-kinetic-urdf \
                        ros-kinetic-rviz \
                        ros-kinetic-kdl-conversions \
                        ros-kinetic-eigen-conversions \
                        ros-kinetic-tf2-sensor-msgs \
                        ros-kinetic-pcl-ros \
                        ros-kinetic-navigation \
                        ros-kinetic-sophus \
                        ros-kinetic-ar-track-alvar-msgs \
                        ros-kinetic-rqt-joint-trajectory-controller \
                        libusb-dev \
                        libftdi-dev \
                        libspnav-dev \
                        libcwiid-dev \
                        libignition-math2-dev
```

## Installation of Gazebo:
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
```


## Quickstart

### 1.Compile all the packages
**Note**: All the command in ROS, I recommand executing in terminal without anaconda, this may lead some interference with your dependencies.

I have alter some github package or version in files, use `gazebo_ros_kinetic.repos` in my github [here](https://github.com/abstractguy/gym_gazebo_kinetic/blob/kinetic/gym_gazebo/envs/installation/gazebo_ros_kinetic.repos), please.
```bash
cd gym_gazebo_kinetic/envs/installation
bash setup_kinetic.bash
```

Put model file into your workspace/src folder.
```bash
cd gym_gazebo_kinetic/envs/installation
bash turtlebot_setup.bash
```

After first two steps above, you will find five lines being added in your `~/.bashrc`:
```bash
source /home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/gym_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/../assets/models
export GYM_GAZEBO_WORLD_MAZE=/home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/../assets/worlds/maze.world
export GYM_GAZEBO_WORLD_CIRCUIT=/home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/../assets/worlds/circuit.world
export GYM_GAZEBO_WORLD_CIRCUIT2=/home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/../assets/worlds/circuit2.world
export GYM_GAZEBO_WORLD_CIRCUIT2C=/home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/../assets/worlds/circuit2c.world
export GYM_GAZEBO_WORLD_ROUND=/home/samuel/school/Project/gym-gazebo/gym_gazebo/envs/installation/../assets/worlds/round.world
```

Here open a new terminal, and uncomment your conda env in your `.bashrc`. Then using it with below:
```bash
source activate gymenv

cd gym_gazebo/examples/scripts_turtlebot
python circuit2_turtlebot_lidar_qlearn.py
```


### 2.Open gazebo
Open another Terminal:
```bash
cd gym-gazebo/gym_gazebo/envs/installation/
source turtlebot_setup.bash
# here the number is set in your code, default 12346
export GAZEBO_MASTER_URI=http://localhost:12346
gzclient
```

Display a graph showing the current reward history by running the following script:
```bash
cd examples/utilities
python display_plot.py
```

## Picture
<p align="center">
  <img src="https://github.com/abstractguy/gym_gazebo_kinetic/blob/kinetic/imgs/qlearn.png"><br><br>
</p>

<p align="center">
  <img src="https://github.com/abstractguy/gym_gazebo_kinetic/blob/kinetic/imgs/dqn.png"><br><br>
</p>


## LICENCE
[MIT License](https://github.com/abstractguy/gym_gazebo_kinetic/blob/kinetic/LICENSE)


## FAQ

**Q1**: If encountering that `ImportError: No module named msg` like below:

```bash
Traceback (most recent call last):
  File "/home/samuel/school/Project/gym_gazebo_kinetic/gym_gazebo/envs/installation/gym_ws/src/hector_gazebo/hector_gazebo_thermal_camera/cfg/GazeboRosThermalCamera.cfg", line 5, in <module>
    from driver_base.msg import SensorLevels
ImportError: No module named msg
```
