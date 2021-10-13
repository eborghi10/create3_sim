FROM ubuntu:20.04
LABEL maintainer="asoragna at irobot dot com"

# working directory
ENV HOME /root
WORKDIR $HOME

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

#######
# Install ROS 2 Foxy
#######

# Setup locale
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Setup sources
RUN apt-get update && apt-get install -y software-properties-common curl gnupg lsb-release
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
RUN apt-get update && apt-get install -y ros-foxy-desktop

RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source /opt/ros/foxy/setup.bash ' >> $HOME/.bashrc

#######
# Install Gazebo
#######

RUN curl -sSL http://get.gazebosim.org | sh

#######
# Install ROS dev tools
#######

RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

RUN rosdep init
RUN rosdep update

#######
# Create3 workspace
#######

