#!/usr/bin/env bash

# File:          docker_install.sh
# Included by:   Samuel Duclos
# For            Myself.
# Description:   Install Docker for Nvidia GPU from scratch.
# Note:          Running line-by-line is recommended.
# Usage:         cd ${HOME}/school/Project/gym_gazebo_kinetic && \
#                    bash docker_install.sh


# Uninstall previous Docker versions.
sudo apt-get remove -y docker docker-engine docker.io containerd runc

# Uninstall Docker Engine, CLI and Containerd packages.
sudo apt-get -y purge docker-ce docker-ce-cli containerd.io

# Purge previous Docker configurations.
sudo rm -rf /var/lib/docker

# Retrieve computer's architecture.
ARCHITECTURE=$(uname -m)
if [[ "${ARCHITECTURE}" == "x86_64" ]]
then
    ARCHITECTURE="amd64"

    # Install Nvidia GPU drivers.
    sudo ubuntu-drivers autoinstall
fi

# Make sure we see latest updates.
sudo apt-get update


# Install Docker Engine the easy way.
#curl -fsSL https://get.docker.com | sudo sh -

# Else, longer Docker Engine install. --->

# Prepare repository addition through HTTPS.
sudo apt-get install -y apt-transport-https \
                        ca-certificates \
                        curl \
                        gnupg-agent \
                        software-properties-common

# Add repository GPG key.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

# Verify GPG key.
sudo apt-key fingerprint 0EBFCD88

# Add architecture's repository to apt.
sudo add-apt-repository "deb [arch=${ARCHITECTURE}] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# Update package descriptors.
sudo apt-get update

# Install Docker Engine.
sudo apt-get install docker-ce docker-ce-cli containerd.io

# <--- ... Longer Docker Engine install.


# Make Docker available to current non-root user without sudo and load at boot-time.
sudo systemctl --now enable docker && \
sudo groupadd docker && \
sudo usermod -aG docker $USER && \
newgrp docker && \
sudo chown "$USER":"$USER" /home/"$USER"/.docker -R && \
sudo chmod g+rwx "$HOME/.docker" -R && \
sudo systemctl enable docker.service && \
sudo systemctl enable containerd.service


# Test new Docker installation.
docker run hello-world


# Install Nvidia GPU stuff.
distribution=$(source /etc/os-release; echo $ID$VERSION_ID) && \
curl -fsSL https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - && \
curl -fsSL https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
    | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Windows users only:
#curl -fsSL https://nvidia.github.io/nvidia-container-runtime/experimental/$distribution/nvidia-container-runtime.list \
#    | sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list

# Nvidia Docker install.
sudo apt-get update && \
sudo apt-get install -y nvidia-docker2 && \
sudo systemctl restart docker


# Test install.
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
