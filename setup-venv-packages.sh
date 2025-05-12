#!/bin/bash -l

pip install omegaconf==2.3.0

# Install other realted libraies (gymnasium_robotics requires pettingzoo and imageio)
pip install msgpack==1.1.0 seaborn rich ruamel.yaml==0.17.32 opencv-python opencv-python-headless tensorflow-datasets datasets colored zmq transforms3d modern-robotics==1.1.1

pip install pybullet
pip install gin-config imageio six # for use in language table