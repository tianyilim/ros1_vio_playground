FROM osrf/ros:noetic-desktop-full

RUN apt-get update

# Some basic utilities
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y \
    gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5 \
    tmux ranger magic htop build-essential git

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bash_profile

RUN apt-get install -y \
        # Base tools
        cmake \
        build-essential \
        git \
        unzip \
        pkg-config \
        python3-dev \
        # ROS dependencies
        python3-catkin-tools python3-osrf-pycommon \
        ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
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
        software-properties-common

# Build OpenCV
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.4.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$nproc && make install && \
    cd / && rm -rf /tmp/opencv

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.6 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 .. && \
    make -j$nproc && make install && \
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

# TUI helpers and config files
COPY docker_build_utils/bashrc /root/.bashrc
COPY docker_build_utils/tmux.conf /root/.tmux.conf
COPY docker_build_utils/vimrc /root/.vimrc

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm-256color
WORKDIR /
CMD ["bash"]
