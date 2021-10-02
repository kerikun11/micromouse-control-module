#!/bin/sh

SRC_DIR=$(dirname $(dirname $(readlink -f $0)))
mkdir -p $SRC_DIR/build

# define docker options according to each env
options=()

# exec user
if [ -f /etc/passwd ]; then
    options=(
        "${options[@]}"
        --volume /etc/group:/etc/group:ro
        --volume /etc/passwd:/etc/passwd:ro
        --volume /etc/shadow:/etc/shadow:ro
        --user $(id -u $USER):$(id -g $USER)
    )
fi

# x11 redirect
if [[ -v DISPLAY ]] && [ -f $HOME/.Xauthority ]; then
    options=(
        "${options[@]}"
        --network host
        --env DISPLAY
        --tmpfs=$HOME
        --volume $HOME/.Xauthority:$HOME/.Xauthority:ro
    )
fi

# docker run
(
    set -x
    docker run --rm --interactive --tty \
        --volume $SRC_DIR:/src \
        --workdir /src/build \
        "${options[@]}" \
        micromouse-control-module \
        "$@" # pass arguments as they are
)
