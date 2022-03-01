#!/bin/bash
docker run --net=host -it --rm \
           -v $(realpath ..):/root/catkin_ws/src/s_graphs \
           -w /root/catkin_ws/src/s_graphs \
           $@ \
           s_graphs
