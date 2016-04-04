rosparam set /use_sim_time true
rosbag play --clock ./calib2.bag
rosrecord - e
rosrun camera_calibration cameracalibrator.py --size 8x5 --square 0.029 --no-service-check image:=/camera/image_raw camera:=/camera
