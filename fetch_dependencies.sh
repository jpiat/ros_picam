#!/bin/sh

sudo apt-get update
sudo apt-get install gcc g++ libx11-dev libxt-dev libxext-dev libgraphicsmagick1-dev libcv-dev

if [ ! -d dependencies ]; then
	mkdir dependencies
fi

cd dependencies

if [ ! -d userland ]; then
	git clone https://github.com/raspberrypi/userland.git
else
	cd userland
	git pull
	cd ..
fi

if [ ! -d robidouille ]; then
	git clone https://github.com/jpiat/robidouille.git
else
	cd robidouille
	git pull
	cd ..
fi

if [ ! -d motor_hat_clib ]; then
	git clone https://github.com/jpiat/motor_hat_clib.git
else
	cd motor_hat_clib
	git pull
	cd ..
fi

cd userland
USERLAND_PWD=${PWD}
./buildme
cd ..

echo ${USERLAND_PWD}
cd robidouille/raspicam_cv
mkdir objs
USERLAND_ROOT=${USERLAND_PWD} make
sudo make install
