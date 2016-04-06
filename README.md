Repository for a smart camera based on raspebrry pi and raspberry camera module.
The sequence cpature folder contains everything to capture raw grayscale image and IMU data with associated timestamps.


To install, run the fetch dependencies script that will install :
- OpenCV3.0 from source
- RaspiCamCV
- required packages


To run the ROS related script you'll need to follow instructions at :
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

and install the sensor_msgs package (run compile part with -j2 argument to avoid compiler errors).

