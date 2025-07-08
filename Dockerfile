FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/noetic
ENV ROS_PACKAGE_PATH=$ROS_ROOT/share
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib
ENV PYTHONPATH=$ROS_ROOT/lib/python3/dist-packages
ENV FLIGHTMARE_PATH=/home/catkin_ws/src/flightmare
ENV PYBIND11_INCLUDE_DIR=/usr/include/pybind11

# ----------------------------
# 1. System dependencies
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    build-essential \
    python3 python3-dev python3-pip \
    cmake \
    git \
    vim \
    curl \
    ca-certificates \
    gnupg2 \
    libzmqpp-dev \
    libopencv-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dev \
    libeigen3-dev \
    pybind11-dev \
    libgoogle-glog-dev 

# ----------------------------
# 2. ROS Noetic installation
# ----------------------------
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key F42ED6FBAB17C654 \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-octomap-msgs \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py \
    ros-noetic-gazebo-plugins \
    ros-noetic-octomap-ros \
    ros-noetic-rqt \
    ros-noetic-web-video-server \
    ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# ----------------------------
# 3. Clone workspace packages
# ----------------------------
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src

RUN git clone https://github.com/catkin/catkin_simple.git && \
    git clone https://github.com/ethz-asl/eigen_catkin.git && \
    git clone https://github.com/ethz-asl/mav_comm.git && \
    git clone https://github.com/ethz-asl/rotors_simulator.git && \
    git clone https://github.com/uzh-rpg/rpg_quadrotor_common.git && \
    git clone https://github.com/uzh-rpg/rpg_quadrotor_control.git && \
    git clone https://github.com/uzh-rpg/rpg_single_board_io.git

# ----------------------------
# 4. Copy Flightmare packages
# ----------------------------
COPY ./flightlib /home/catkin_ws/src/flightmare/flightlib
COPY ./flightrl /home/catkin_ws/src/flightmare/flightrl
COPY ./flightros /home/catkin_ws/src/flightmare/flightros

# Clean old externals if any
RUN rm -rf /home/catkin_ws/src/flightmare/flightlib/externals/*

# ----------------------------
# 5. Install flightlib and flightrl
# ----------------------------
RUN pip3 install --upgrade  pip

WORKDIR /home/catkin_ws/src/flightmare/flightlib
RUN pip3 install .

WORKDIR /home/catkin_ws/src/flightmare/flightrl
RUN pip3 install .

RUN rm -rf /root/.cache/pip

# ----------------------------
# 6. Build catkin workspace
# ----------------------------
WORKDIR /home/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


# ----------------------------
# 7. Set environment for containers
# ----------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

# ----------------------------
# 8. Default entrypoint
# ----------------------------
CMD ["/bin/bash"]
