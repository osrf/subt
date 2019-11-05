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
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
        lsb-release \
        mercurial \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-pip \
        python3-venv \
        ruby2.5 \
        ruby2.5-dev \
        software-properties-common \
        sudo \
        vim \
        wget \
        net-tools \
        iputils-ping \
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

# install ROS and required packages
RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo apt-get update \
 && sudo apt-get install -y \
    python-catkin-tools \
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
 && sudo rosdep init \
 && sudo apt-get clean

RUN rosdep update

# sdformat8-sdf conflicts with sdformat-sdf installed from gazebo
# so we need to workaround this using a force overwrite
# Do this before installing ign-gazebo
RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && sudo /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -' \
 && sudo /bin/sh -c 'apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'

# install ign-blueprint
RUN sudo apt-get update \
 && sudo apt-get install -y \
    ignition-blueprint \
 && sudo apt-get clean

# install the ros to ign bridge
RUN sudo apt-get update \
 && sudo apt-get install -y \
    ros-melodic-ros1-ign \
 && sudo apt-get clean

# Clone all the subt models so that you don't download them every time
# docker is run
RUN mkdir -p subt_ws/src \
 && cd subt_ws/src \
 && hg clone https://bitbucket.org/osrf/subt -b urban_circuit

# Download virtual stix models
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Edgar Mine Virtual STIX" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Base Station" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Black and Decker Cordless Drill" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Fiducial" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Fire Extinguisher" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/JanSport Backpack Red" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Jersey Barrier" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Rescue Randy Sitting" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Samsung J8 Black" -v 4

# Download the robot models
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 1" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 2" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 3" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 4" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X1 Config 5" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 1" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 2" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 3" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 4" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 5" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X2 Config 6" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 1" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 2" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 3" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X3 UAV Config 4" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 1" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 2" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 3" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 4" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/X4 UAV Config 5" -v 4

# Download the tunnel circuit models
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/subt_tunnel_staging_area" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 1" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 2" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 3" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 4" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 5" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 6" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile 7" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Tunnel Tile Blocker" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Constrained Tunnel Tile Short" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Constrained Tunnel Tile Tall" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile 4-way Intersection" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile 90-degree Turn" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile Straight" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile Ramp" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Rough Tunnel Tile Vertical Shaft" -v 4

# Download urban circuit tiles
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Superpose" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Service Room" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Straight Door Right" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Straight Door Left" -v 4
# RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Stairwell Straight" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Starting Area" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Straight" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban 3-Way Right Intersection" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Bend Left" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Bend Right" -v 4
# RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban 4way Intersection" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Stairwell Platform" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Platform" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Platform Open" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Straight Door Left Flipped" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Straight Door Right Flipped" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Elevation Up" -v 4
RUN ign fuel download --url "https://fuel.ignitionrobotics.org/openrobotics/models/Urban Elevation Down" -v 4

WORKDIR /home/$USERNAME/subt_ws

# Install Rotors
# RUN wget https://s3.amazonaws.com/osrf-distributions/subt_robot_examples/releases/subt_robot_examples_latest.tgz
# RUN tar xvf subt_robot_examples_latest.tgz

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'

RUN /bin/sh -c 'echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/subt_ws/install/setup.sh" >> ~/.bashrc'

# Create a directory required by cloudsim
RUN mkdir -p /tmp/ign

# Copy entry point script, and set the entrypoint
COPY run_sim.bash ./
ENTRYPOINT ["./run_sim.bash"]

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
