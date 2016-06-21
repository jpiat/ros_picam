Repository for a smart camera based on raspebrry pi and raspberry camera module.
The sequence cpature folder contains everything to capture raw grayscale image and IMU data with associated timestamps.


To install, run the fetch dependencies script that will install :
- OpenCV3.1.0 from source
- RaspiCamCV
- required packages


To run the ROS related script you'll need to follow instructions at :
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

and install the sensor_msgs package (run compile part with -j2 argument to avoid compiler errors).

Material Required :
- Raspberry-pi2/3
- Raspberry camera module (or any of the clone, the one with m12 lens mount are great)
- MPU9250 IMU module (or MPU9150)
- HDMI extender for raspberry-pi camera (look on tindie) if you want to have camera far from Pi
- class 10 sd card
- fast USB storage (if you don't want your image to be stored on the Pi sd card)
- push-button with dupont wire (to trigger captures)
- LED with 220ohm resistor and dupont wires (to see when a capture is on-going)
