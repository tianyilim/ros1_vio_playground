FROM osrf/ros:noetic-desktop-full

RUN apt-get update

# Some basic utilities
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y \
    gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5 \
    tmux ranger magic htop build-essential git

# Base tools
RUN apt-get install -y \
    cmake \
    build-essential \
    git \
    unzip \
    pkg-config \
    python3-dev \
    # ROS dependencies
    python3-catkin-tools python3-osrf-pycommon \
    ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-message-filters \
    ros-noetic-hector-trajectory-server \
    # OpenCV dependencies
    python3-numpy \
    # Pangolin dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libpython3-dev \
    libeigen3-dev \
    apt-transport-https \
    ca-certificates\
    software-properties-common \
    # Ceres dependencies
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    # OpenCV dependencies
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev

# Build Ceres 1.14.0 for VINS-MONO
WORKDIR /src
RUN wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz && \
    mkdir /src/ceres-bin && \
    tar zxf ceres-solver-1.14.0.tar.gz --no-same-owner && \
    cd /src/ceres-bin && \
    cmake /src/ceres-solver-1.14.0 -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j8 && make install && \
    rm -rf /src/ceres-solver-1.14.0.tar.gz /src/ceres-bin

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.4.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/opencv

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.6 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/Pangolin

# Setup ros entrypoint and post create command that compiles everything
COPY docker_build_utils/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker_build_utils/post_create_command.sh /post_create_command.sh
RUN chmod +x /ros_entrypoint.sh
RUN chmod +x /post_create_command.sh
ENV ROS_DISTRO noetic
ENV LANG en_US.UTF-8

# Setup catkin ws
RUN mkdir -p /catkin_ws/src /catkin_ws/build /catkin_ws/devel /catkin_ws/install /ORB_SLAM3

# Setup orbslam
WORKDIR /
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git && cd ORB_SLAM3 && \
    cd Vocabulary && tar -xvf ORBvoc.txt.tar.gz && cd .. && \
    ./build.sh

# Clone orbslam ros wrapper
WORKDIR /catkin_ws/src
RUN git clone https://github.com/thien94/orb_slam3_ros_wrapper

# Setup VINS-Fusion
WORKDIR /catkin_ws/src
RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git

# Setup OpenVINS
WORKDIR /catkin_ws/src
RUN git clone https://github.com/rpng/open_vins/

# Ensure all ROS dependencies are installed
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths /catkin_ws/src --ignore-src -r -y

# Modify the catkin workspace to the specific location of the orbslam3 wrapper
# And copy vocabulary over
WORKDIR /catkin_ws/src
RUN sed -i 's/$ENV{HOME}\/Packages//' orb_slam3_ros_wrapper/CMakeLists.txt && \
    cp /ORB_SLAM3/Vocabulary/ORBvoc.txt orb_slam3_ros_wrapper/config/ORBvoc.txt

# Build the catkin workspace, sourcing in `sh` syntax
WORKDIR /catkin_ws
RUN . /opt/ros/noetic/setup.sh && catkin build

# TUI helpers and config files
COPY docker_build_utils/bashrc /root/.bashrc
COPY docker_build_utils/tmux.conf /root/.tmux.conf
COPY docker_build_utils/vimrc /root/.vimrc
COPY docker_build_utils/bash_profile_ext /tmp/bash_profile_ext
RUN cat /tmp/bash_profile_ext >> /root/.bash_profile && rm /tmp/bash_profile_ext

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm-256color
WORKDIR /
CMD ["bash"]
