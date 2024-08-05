# ORB_SLAM3 docker

This docker is based on Ros Noetic Ubuntu 20. It's based off the OSRF ROS Noetic Desktop image.

> **NOTE:** As symlinks are not supported in Docker, it's difficult to mount datasets outside the `Datasets` folder. As a stopgap, Tianyi mounted an absolute path on his local machine [in the build file](./build_container.sh). Modifiy it before building!

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

Code modifications can be done in the `code` folder.
