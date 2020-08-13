#!/bin/bash

# Get packages required for OpenCV
# get a fresh start
$ sudo apt-get update
$ sudo apt-get upgrade
# remove old versions, if not placed in a virtual environment (let pip search for them)
$ sudo pip uninstall tensorflow
$ sudo pip3 uninstall tensorflow
# install the dependencies (if not already onboard)
$ sudo apt-get install gfortran
$ sudo apt-get install libhdf5-dev libc-ares-dev libeigen3-dev
$ sudo apt-get install libatlas-base-dev libopenblas-dev libblas-dev
$ sudo apt-get install liblapack-dev cython
$ sudo pip3 install pybind11
$ sudo pip3 install h5py
# upgrade setuptools 40.8.0 -> 47.1.1
$ sudo pip3 install --upgrade setuptools
# install gdown to download from Google drive
$ pip install gdown
# set PATH
$ export PATH=$PATH:/home/pi/.local/bin
# download the wheel
$ gdown https://drive.google.com/uc?id=11mujzVaFqa7R1_lB7q0kVPW22Ol51MPg
# install TensorFlow
$ sudo -H pip3 install tensorflow-2.2.0-cp37-cp37m-linux_armv7l.whl
# and complete the installation by rebooting
$ reboot

sudo apt-get -y install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install qt4-dev-tools libatlas-base-dev

# Need to get an older version of OpenCV because version 4 has errors
sudo pip3 install opencv-python==3.4.6.27
sudo apt-get install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev  libqtgui4  libqt4-test
# Get packages required for TensorFlow
# Using the tflite_runtime packages available at https://www.tensorflow.org/lite/guide/python
# Will change to just 'pip3 install tensorflow' once newer versions of TF are added to piwheels

# pip3 install tensorflow


# pyfirmata interaction between python and arduino board
sudo pip3 install pyfirmata

# not entirely sure why we need this, but it fixed some things
sudo apt-get install libxml2-dev libxslt-dev
# drone communication between pixhawk and raspberry pi for positioning and attitude
# took a WHILE on my rpi
sudo pip3 install dronekit
