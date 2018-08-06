# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

# Tools I find useful during development
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        libbluetooth-dev \
        libcwiid-dev \
        libspnav-dev \
        libusb-dev \
        lsb-release \
        mercurial \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-pip \
        python3-venv \
        software-properties-common \
        sudo \
        vim \
        wget \
 && apt-get clean

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean

RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && sudo apt-get update \
 && sudo apt-get install -y \
    libgazebo9-dev \
    gazebo9 \
    python-catkin-tools \
    python-rosinstall \
    ros-melodic-desktop-full \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-twist-mux \
 && sudo rosdep init \
 && sudo apt-get clean

RUN rosdep update

RUN mkdir -p subt_ws/src \
 && cd subt_ws/src \
 && hg clone https://bitbucket.org/osrf/subt

RUN /bin/sh -c 'echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
