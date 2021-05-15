#!/bin/sh

SRC_DIR=$(dirname $(dirname $(readlink -f $0)))
mkdir -p $SRC_DIR/build

docker run --rm -it \
    -v $SRC_DIR:/src \
    -w /src/build \
    -v /etc/group:/etc/group:ro \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/shadow:/etc/shadow:ro \
    -u $(id -u $USER):$(id -g $USER) \
    -e DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    --network host \
    micromouse-control-module \
    "$@" # pass arguments as they are
