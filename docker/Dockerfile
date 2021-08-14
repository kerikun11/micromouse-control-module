# base image
FROM ubuntu:20.04

# description
LABEL maintainer="Ryotaro Onuki <kerikun11+github@gmail.com>"

# install packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    doxygen \
    graphviz \
    lcov \
    libgtest-dev \
    python3-dev \
    python3-distutils \
    python3-matplotlib \
    python3-pybind11 \
    python3-tk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
