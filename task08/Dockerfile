FROM ros:jazzy-ros-base-noble@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6

RUN apt-get update && apt-get install --yes --no-install-recommends \
    libeigen3-dev \
    libgraphicsmagick++1-dev \
    libyaml-cpp-dev \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-pytest \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-launch-testing \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-util \
    ros-jazzy-yaml-cpp-vendor \
    && rm -rf /var/lib/apt/lists/*
