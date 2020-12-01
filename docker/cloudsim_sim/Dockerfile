# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM osrf/subt-virtual-testbed:models_latest

# Tools I find useful during development
RUN sudo apt-get update -qq \
 && sudo apt-get install --no-install-recommends -y -qq \
        build-essential \
        bwm-ng \
        atop \
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
        xvfb \
 && sudo apt-get clean -qq

# Install AWS CLI. This is needed by cloudsim to capture ROS logs.
RUN pip3 install --upgrade awscli=="1.16.220"
VOLUME /root/.aws

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
 && rosdep init \
 && apt-get clean

# sdformat8-sdf conflicts with sdformat-sdf installed from gazebo
# so we need to workaround this using a force overwrite
# Do this before installing ign-gazebo
RUN /bin/sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && /bin/sh -c 'apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'

# install ign-blueprint
RUN apt-get update \
&&  apt-get install -y \
    ignition-blueprint \
 && apt-get clean

# install the ros to ign bridge
RUN apt-get update \
 && apt-get install -y \
    ros-melodic-ros-ign \
 && apt-get clean -qq

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

RUN rosdep update

# Download the public models
RUN ign fuel download -v 4 -j 8 --type model -u "https://fuel.ignitionrobotics.org/OpenRobotics/collections/SubT Tech Repo"

# Download niosh models, set 1.
COPY download_niosh_1.sh ./
RUN ./download_niosh_1.sh

# Download niosh models, set 2.
COPY download_niosh_2.sh ./
RUN ./download_niosh_2.sh

# Download niosh models, set 3.
COPY download_niosh_3.sh ./
RUN ./download_niosh_3.sh

# Clone all the subt models so that you don't download them every time
# docker is run
RUN mkdir -p subt_ws/src \
 && cd subt_ws/src \
 && git clone https://github.com/osrf/subt subt_ws/src/subt

WORKDIR /home/$USERNAME/subt_ws

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'

RUN /bin/sh -c 'echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/subt_ws/install/setup.sh" >> ~/.bashrc'

# Create a directory required by cloudsim
RUN mkdir -p /tmp/ign

# Copy entry point script, and set the entrypoint
COPY cloudsim_sim/run_sim.bash ./
ENTRYPOINT ["./run_sim.bash"]

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
