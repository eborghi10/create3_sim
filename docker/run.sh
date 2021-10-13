#!/bin/bash
#
# @author Alberto Soragna (asoragna at irobot dot com)
# @2020

docker run -it --rm \
	  --net=host \
	  --privileged \
	  -v $PWD/..:/root/create3_sim \
	  create3_foxy \
	  bash
