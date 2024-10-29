#!/bin/bash
set -e

# Remove existing container
docker rm -f orbslam3 &>/dev/null || true

# Docker build
docker buildx build -t kimera:ubuntu20_noetic_cuda -f Dockerfile_Kimera .
