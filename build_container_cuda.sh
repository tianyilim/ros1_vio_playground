# checking if you have nvidia
if ! nvidia-smi | grep "Driver" 2>/dev/null; then
  echo "******************************"
  echo """It looks like you don't have nvidia drivers running. Consider running build_container_cpu.sh instead."""
  echo "******************************"
  while true; do
    read -p "Do you still wish to continue?" yn
    case $yn in
      [Yy]* ) make install; break;;
      [Nn]* ) exit;;
      * ) echo "Please answer yes or no.";;
    esac
  done
fi

# Ensure that ORB_SLAM3 is checked out
git submodule update --init --recursive # should clone from latest orbslam3
cd ORB_SLAM3/Vocabulary && tar -xvf ORBvoc.txt.tar.gz && cd - || exit 1 # Extract vocabulary
# Modify the catkin workspace to the specific location of the orbslam3 wrapper
sed -i 's/$ENV{HOME}\/Packages//' orb_slam3_ros_wrapper/CMakeLists.txt
# Copy vocabulary over
cp ORB_SLAM3/Vocabulary/ORBvoc.txt orb_slam3_ros_wrapper/config/ORBvoc.txt

# UI permisions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

xhost +local:docker
# docker pull jahaniam/orbslam3:ubuntu20_noetic_cuda

# Remove existing container
docker rm -f orbslam3 &>/dev/null
[ -d "ORB_SLAM3" ] && sudo rm -rf ORB_SLAM3 && mkdir ORB_SLAM3

# Copy some stuff to the docker build context
mkdir docker_build_utils
cp ~/.bashrc docker_build_utils/
cp ~/.tmux.conf docker_build_utils/
cp ~/.vimrc docker_build_utils/

docker buildx build -t orbslam3 -f Dockerfile.cuda .

# Create a new container
docker run -td --privileged --net=host --ipc=host \
    --name="orbslam3" \
    --gpus=all \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "XAUTHORITY=$XAUTH" \
    -e ROS_IP=127.0.0.1 \
    --cap-add=SYS_PTRACE \
    -v $XSOCK:$XSOCK \
    -v `pwd`/Datasets:/Datasets \
    -v /etc/group:/etc/group:ro \
    -v `pwd`/ORB_SLAM3:/ORB_SLAM3 \
    --mount type=bind,source="$HOME/mt/large_scale_pgo",target=/large_scale_pgo \
    --mount type=bind,source="`pwd`/orb_slam3_ros_wrapper",target=/catkin_ws/src/orb_slam3_ros_wrapper \
    jahaniam/orbslam3:ubuntu20_noetic_cuda bash

# Git pull orbslam and compile
# docker exec -it orbslam3 bash -i -c  "git clone -b add_euroc_example.sh https://github.com/jahaniam/ORB_SLAM3.git /ORB_SLAM3 && cd /ORB_SLAM3 && chmod +x build.sh && ./build.sh "
