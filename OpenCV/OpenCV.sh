#!/bin/bash

cd 
git clone https://github.com/jayrambhia/Install-OpenCV.git
cd Install-OpenCV
cd Ubuntu
sudo ./opencv_latest.sh
cd
sudo rm -rf Install-OpenCV
pip install imutils
