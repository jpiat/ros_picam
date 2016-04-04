import time, sys, os, math
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

import ImageFile

model = 'plumb_bob'

D = [0.09588004493901636, -0.2751577208590459, 0.0007265931787498115, -0.007979244347949572, 0.0]
K = [627.4842111637764, 0.0, 301.42612443545056, 0.0, 626.5832675966146, 249.65402637073538, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [629.0787353515625, 0.0, 296.5435491975986, 0.0, 0.0, 632.731201171875, 249.43851519734744, 0.0, 0.0, 0.0, 1.0, 0.0]

G_2_MPSS=9.80665
ACC_RANGE=4.0
GYRO_RANGE=500.0

def acc_raw_to_ms(d):
	g = (ACC_RANGE/32768.0)*float(d)
	g = g*G_2_MPSS #convert to m/s^-2
	return g

def gyro_raw_to_rads(d):
	g = (GYRO_RANGE/32768.0)*float(d) # deg/s
	g = math.radians(g)#g*(math.pi/180.0)#convert to radian/s
	return g

def create_vector3(x, y, z):
	v = Vector3()
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
	Img.encoding = "mono8" #needs to be changed to rgb8 for rgb images
	Img.step=Img.width #some nodes may complains ...
	Img.header.frame_id = "camera"
	Img_data = list(im.getdata()) #works for mono channels images (grayscale)
	#Img_data = [pix for pix in im.getdata()]
	#  Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
	Img.data = Img_data
	return (im_stamp, Stamp, Img)

def GetCameraInfo(width, height):
	cam_info = CameraInfo()
	cam_info.width = width
	cam_info.height = height
	cam_info.distortion_model = model
	#cam_info.D = [0.0]*5
	#cam_info.K = [0.0]*9
	#cam_info.R = [0.0]*9
	#cam_info.P = [0.0]*12
	cam_info.D = D
        cam_info.K = K
        cam_info.R = R
        cam_info.P = P
	cam_info.binning_x = 0
	cam_info.binning_y = 0
	return cam_info

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
    print 'found '+str(len(all))+' images'
    return all

def CreateMonoBag(imgs,imu,bagname):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')
    fimu = open(imu, "r")
    line = "%"
    while line[0] == '%': #skipping comment lines
	line = fimu.readline()
    try:
	print 'Complete 0% \r',
        for i in range(len(imgs)):
            #print("Adding %s" % imgs[i])
            (im_stamp, Stamp, Img) = GetImageFromFile(imgs[i])
            while line :
		(ts, ImuStamp, imu) = GetImuFromLine(line)
		bag.write('imu', imu, ImuStamp)
		if ts > im_stamp:
			break
		line = fimu.readline()   
            print 'Complete '+str((float(i)/float(len(imgs)))*100.0)+'% \r' ,
            cam_info = GetCameraInfo(Img.width, Img.height)
            bag.write('camera/camera_info', cam_info, Stamp)
            bag.write('camera/image_raw', Img, Stamp)
        print 'Complete 100% \n\r'
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
