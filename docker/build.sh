#!/bin/sh

## config
set -x # show command

## build
docker build --tag micromouse-control-module $(dirname $0)
