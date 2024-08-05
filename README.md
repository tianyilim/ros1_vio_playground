# ORB_SLAM3 docker

This docker is based on Ros Noetic Ubuntu 20. It's based off the OSRF ROS Noetic Desktop image. For now some link paths are hardcoded to Tianyi's MT thesis folder. Check [the build script](./build_container.sh) for details.

## Quickstart:

### Building

```bash
./build_container.sh
```

This builds the dockerfile, and as a post-installation step, compiles ORB_SLAM3 and the ORB_SLAM3 ROS wrapper.

### Running

```bash
docker start orbslam3; docker attach orbslam3
```

This should attach you into an interactive `bash` terminal where you can do stuff.

`tmux` is installed, but the leader is `C-a` to avoid conflicts with nested `tmux` sesisons.

## Legacy from fork

## Compilation and Running

Steps to compile the Orbslam3 on the sample dataset:

- `./download_dataset_sample.sh`
- `build_container_cpu.sh` or `build_container_cuda.sh` depending on your machine.

Now you should see ORB_SLAM3 is compiling.

- Download A sample MH02 EuRoC example and put it in the `Datasets/EuRoC/MH02` folder

```
mkdir -p Datasets/EuRoC
wget -O Datasets/EuRoC/MH_02_easy.zip http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_02_easy/MH_02_easy.zip
unzip Datasets/EuRoC/MH_02_easy.zip -d Datasets/EuRoC/MH02
```

To run a test example:

- `docker exec -it orbslam3 bash`
- `cd /ORB_SLAM3/Examples && bash ./euroc_examples.sh`
  It will take few minutes to initialize. Pleasde Be patient.

---

You can use vscode remote development.

- `docker exec -it orbslam3 bash`
