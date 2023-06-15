#!/bin/bash

export CROSS_COMPILE=arm-linux-gnueabi-
export ARCH=arm

make -C ~/Build/linux-5.8 M=$PWD
