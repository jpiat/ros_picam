#!/bin/sh

gpio mode 0 up

while true
do
	gpio wfi 0 falling
	now=`date +%H_%M_%S_%d_%m_%Y`
	mkdir -p ~/Pictures/${now}
	./capture_sequence ~/Pictures/${now}
	CAPTURE_PID=$!
	echo $CAPTURE_PID
	sleep 1
	gpio wfi 0 falling
	echo "Ending capture"
	echo $CAPTURE_PID
	pkill -TERM -P $CAPTURE_PID
	cd ~/Pictures
        tar cf ${now}.tar --remove-files ${now}
	sleep 10
done
