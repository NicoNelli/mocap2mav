#!/usr/bin/env sh
#Get eigen, deps and lcm
sudo apt-get install libglib2.0-dev &&
sudo apt-get install libeigen3-dev &&
sudo apt-get install build-essential &&
cd libs/lcm &&
./bootstrap.sh &&
./configure &&
make &&
sudo make install &&
cd .. &&
cd rudeconfig-5.0.5 &&
./configure &&
make &&
sudo make install &&
sudo ldconfig

