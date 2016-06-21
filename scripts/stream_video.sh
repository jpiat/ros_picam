#!/bin/bash

if test -z "$1" 
then
     echo "Not enough arguments "
     echo "Usage : stream_video.sh <dest_ip>"
     exit
fi

echo "Please install netcat on first launch : sudo apt-get netcat"
echo "Copy the following to your host computer console :"
echo "nc -l -p 5000 | mplayer -fps 60 -cache 1024 -"
read -p "Press key when done... " -n1 -s
raspivid -t 0 -b 5000000 -w 1296 -h 972 -o - -fps 30 | nc $1 5000 

