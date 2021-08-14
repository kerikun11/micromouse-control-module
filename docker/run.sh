#!/bin/sh

SRC_DIR=$(dirname $(dirname $(readlink -f $0)))
mkdir -p $SRC_DIR/build

docker run --rm --interactive --tty \
    --volume $SRC_DIR:/src \
    --workdir /src/build \
    --volume /etc/group:/etc/group:ro \
    --volume /etc/passwd:/etc/passwd:ro \
    --volume /etc/shadow:/etc/shadow:ro \
    --user $(id -u $USER):$(id -g $USER) \
    --env DISPLAY \
    --volume $HOME/.Xauthority:$HOME/.Xauthority:ro \
    --network host \
    micromouse-control-module \
    "$@" # pass arguments as they are
