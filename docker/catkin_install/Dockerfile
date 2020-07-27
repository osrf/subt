# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get -q -y install --no-install-recommends tzdata && \
  rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get -q -y install --no-install-recommends \
  # dirmngr \
  lsb-release \
  # gnupg2 is needed for apt-key
  gnupg2 \
  wget \
  && rm -rf /var/lib/apt/lists/*

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get -q -y install --no-install-recommends \
  build-essential \
  cmake \
  ignition-blueprint \
  libusb-dev \
  lsb-release \
  pkg-config python \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-melodic-desktop \
  ros-melodic-joy \
  ros-melodic-robot-localization \
  ros-melodic-ros-control \
  ros-melodic-ros-ign \
  ros-melodic-rotors-control \
  ros-melodic-teleop-twist-keyboard \
  ros-melodic-tf2-sensor-msgs \
  ros-melodic-twist-mux \
  && rm -rf /var/lib/apt/lists/*

# Add new sudo user (based on https://wiki.ros.org/docker/Tutorials/GUI#The_isolated_way)
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME dev
RUN useradd -m $USERNAME && \
  echo "$USERNAME:$USERNAME" | chpasswd && \
  usermod --shell /bin/bash $USERNAME && \
  usermod -aG sudo $USERNAME && \
  echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  usermod  --uid ${user_id} $USERNAME && \
  groupmod --gid ${user_id} $USERNAME

USER ${USERNAME}

# create a workspace in the user's home directory
# (this is where the subt repo can be mounted as a volume)
WORKDIR /home/${USERNAME}
RUN mkdir -p subt_ws/src

CMD [ "/bin/bash" ]