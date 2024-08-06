#!/bin/bash
set -e

# Remove existing container
docker rm -f orbslam3 &>/dev/null

# Docker build
docker buildx build -t orbslam3:ubuntu20_noetic_cuda -f Dockerfile .
