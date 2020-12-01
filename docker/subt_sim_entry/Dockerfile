# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM osrf/subt-virtual-testbed:models_latest

# Tools I find useful during development
RUN sudo apt-get update -qq \
 && sudo apt-get install --no-install-recommends -y -qq \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
        libfcl-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
        lsb-release \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        ruby2.5 \
        ruby2.5-dev \
        software-properties-common \
        vim \
        net-tools \
        iputils-ping \
        libyaml-cpp-dev \
 && sudo apt-get clean -qq


# install ROS and required packages
RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo apt-get update -qq \
 && sudo apt-get install -y -qq \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    ros-melodic-desktop \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-twist-mux \
    ros-melodic-rviz-imu-plugin \
    ros-melodic-rotors-control \
    ros-melodic-ros-ign \
 && sudo rosdep init \
 && sudo apt-get clean -qq

ARG user_id
ENV USERNAME developer
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

RUN rosdep update

# Clone all the subt models so that you don't download them every time
# docker is run
RUN mkdir -p subt_ws/src \
 && git clone https://github.com/osrf/subt subt_ws/src/subt

# Download niosh models, set 1.
COPY download_niosh_1.sh ./
RUN ./download_niosh_1.sh

# Download niosh models, set 2.
COPY download_niosh_2.sh ./
RUN ./download_niosh_2.sh

# Download niosh models, set 3.
COPY download_niosh_3.sh ./
RUN ./download_niosh_3.sh

WORKDIR /home/$USERNAME/subt_ws

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'

RUN /bin/sh -c 'echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/subt_ws/install/setup.sh" >> ~/.bashrc'

# Create a directory required by cloudsim
RUN mkdir -p /tmp/ign

# Copy entry point script, and set the entrypoint
COPY subt_sim_entry/run_sim.bash ./
ENTRYPOINT ["./run_sim.bash"]

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
