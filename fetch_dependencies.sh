#!/bin/sh

sudo apt-get install gcc g++ libx11-dev libxt-dev libxext-dev libgraphicsmagick1-dev libcv-dev libhig$

git clone https://github.com/jpiat/robidouille.git
git clone https://github.com/jpiat/motor_hat_clib.git

git clone https://github.com/raspberrypi/userland.git
cd userland
./buildme

cd 
