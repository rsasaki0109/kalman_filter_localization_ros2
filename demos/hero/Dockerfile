# Pin the official ROS base so a tag update cannot silently change the demo.
FROM ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410

# The base already supplies the ROS dependencies. Only Ubuntu packages are needed.
# Keep APT signature verification enabled; do not contact unused ROS repositories.
RUN apt-get -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/ubuntu.sources \
      -o Dir::Etc::sourceparts=- update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      python3-pil python3-numpy ffmpeg fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

ENV CMAKE_BUILD_PARALLEL_LEVEL=2
WORKDIR /repo
CMD ["bash"]
