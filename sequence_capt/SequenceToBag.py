import time, sys, os, math
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

import ImageFile


def acc_raw_to_ms(d):
	g = (4.0/32768.0)*float(d)
	g = g*9.80665
	return g

def gyro_raw_to_rads(d):
	g = (500.0/32768.0)*float(d) # deg/s
	g = g*(math.pi/180.0)
	return g

def create_vector3(x, y, z):
	v = Vector3()
	# I guess the units will be like that, but I don't know
	v.x = x
	v.y = y
	v.z = z	
	return v

def create_diag_mat(x, y, z):
	mat = [0] * 9
	mat[0] = x
	mat[4] = y
	mat[8] = z
	return mat


def GetImuFromLine(line):
	(ts, ax, ay, az, gx, gy, gz) = [t(s) for t,s in zip((int,int, int, int, int, int, int),line.split())]
	imu = Imu()
	ts = float(ts)/1000000.0
	ImuStamp = rospy.rostime.Time.from_sec(ts)
	imu.angular_velocity = create_vector3(gyro_raw_to_rads(gx), gyro_raw_to_rads(gy), gyro_raw_to_rads(gz))
	imu.angular_velocity_covariance = create_diag_mat(gyro_raw_to_rads(1.0), gyro_raw_to_rads(1.0), gyro_raw_to_rads(1.0));
	imu.linear_acceleration = create_vector3(acc_raw_to_ms(ax), acc_raw_to_ms(ay), acc_raw_to_ms(az))
	imu.linear_acceleration_covariance = create_diag_mat(acc_raw_to_ms(1.0), acc_raw_to_ms(1.0), acc_raw_to_ms(1.0))
	return (ts, ImuStamp, imu)

def GetImageFromFile(im_path):
	fp = open( im_path, "r" )
	p = ImageFile.Parser()
	while 1:
		s = fp.read(307218) # trying to read a full file in one shot ...
		if not s:
		    break
		p.feed(s)
	im = p.close() # we should now have an image object
	im_stamp = os.path.basename(im_path).split(".")[0] #image timestamp is directly encoded in file name
	im_stamp =  float(im_stamp)/1000000.0
	Stamp = rospy.rostime.Time.from_sec(im_stamp)
	Img = Image()
	Img.header.stamp = Stamp
	Img.width = im.size[0]
	Img.height = im.size[1]
	Img.encoding = "mono8"
	Img.header.frame_id = "camera"
	Img_data = [pix for pix in im.getdata()]
	#  Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
	Img.data = Img_data
	return (im_stamp, Stamp, Img)

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            files.sort()
            for f in files:
                if os.path.splitext(f)[1] in ['.pgm']:
                    all.append( os.path.join( path, f ) )
    return all

def CreateMonoBag(imgs,imu,bagname):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')
    fimu = open(imu, "r")
    line = "%"
    while line[0] == '%': #skipping comment lines
	line = fimu.readline()
    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            (im_stamp, Stamp, Img) = GetImageFromFile(imgs[i])
            while True :
                (ts, ImuStamp, imu) = GetImuFromLine(line)
		bag.write('imu', imu, ImuStamp)
		if ts > im_stamp:
			break
		line = fimu.readline()   

            bag.write('camera/image_raw', Img, Stamp)
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs = GetFilesFromDir(args[0])
    imu_file = args[0]+"/IMU.log"
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()
    CreateMonoBag(all_imgs, imu_file,args[1])        

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag( sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir bagfilename")
