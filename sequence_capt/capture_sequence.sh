#!/bin/bash

now=`date +%H_%M_%S_%d_%m_%Y`
mkdir -p ~/Pictures/${now}
#./capture_sequence $1 ~/Pictures/${now}
cd ~/Pictures
tar cf ${now}.tar --remove-files ${now}
