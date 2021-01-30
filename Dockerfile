# Install Docker like this:
#curl -sSL https://get.docker.com | sh
#sudo usermod -aG docker $(whoami)
#sudo shutdown -r now

#ARG NAME=ros_ws
ARG NAME=catkin_ws

FROM ros:kinetic-robot-xenial
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y bash-completion \
                       build-essential \
                       cmake \
                       git \
                       sudo \
                       wget && \
    rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ros
RUN usermod -a -G dialout ros
ADD config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && \
    chown root:root /etc/sudoers.d/99_aptget
#    mkdir -p config && \
#    echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget

ENV USER ros
USER ros
ENV HOME /home/${USER}

# Update packages and install dependencies
RUN sudo sh -c "apt-get update && \
                apt-get install -y ros-kinetic-rviz && \
                rm -rf /var/lib/apt/lists/*"

# Install Gazebo 7 .
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo sh -c "apt-get update && \
                apt-get install -y gazebo7 \
                                   libignition-math2-dev \
                                   ros-kinetic-gazebo-ros-pkgs \
                                   ros-kinetic-gazebo-ros-control && \
                                   #ros-kinetic-ur-gazebo \
                                   #ros-kinetic-ur5-moveit-config \
                                   #ros-kinetic-ur-kinematics \
                                   #moveit_simple_controller_manager \
                rm -rf /var/lib/apt/lists/*"

RUN mkdir -p ${HOME}/${NAME}/src
WORKDIR ${HOME}/${NAME}/src
#RUN /bin/bash -c "git clone https://github.com/abstractguy/UArmForROS.git"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace"
WORKDIR ${HOME}/${NAME}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make"

# Set missing environment variable needed to run Qt applications.
ENV QT_X11_NO_MITSHM 1

# Source bash.
#RUN source ${HOME}/${NAME}/devel/setup.bash

COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc; sudo chown ros /sbin/update_bashrc; sync; /bin/bash -c /sbin/update_bashrc; sudo rm /sbin/update_bashrc

COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh; sudo chown ros /ros_entrypoint.sh

RUN sudo sh -c "apt-get clean && \
                rm -rf /var/lib/apt/lists/*"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["source devel/setup.bash && /bin/bash"]
