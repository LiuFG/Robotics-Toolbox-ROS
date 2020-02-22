#!/usr/bin/python2
# -*- coding: utf-8 -*-

"""
    online segmentation using .npy & SqueezeSeg model

    this script can
                    1. read all .npy file from lidar_2d folder
                    2. predict label from SqueezeSeg model using tensorflow
                    3. publish to 'sqeeuze_seg/points' topic

"""
import sys
import os.path
import numpy as np
from PIL import Image
from numpy import *
import os 
import math
import tensorflow as tf

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header

from squeezeseg.config import *
from squeezeseg.nets import SqueezeSeg
from squeezeseg.utils.util import *
from squeezeseg.utils.clock import Clock
from squeezeseg.imdb import kitti  # ed: header added
#测时间
import datetime

class NPY_TENSORFLOW_TO_ROS():
    def point_cloud_to_panorama(self,points,
                            v_res=0.42,
                            h_res = 0.35,
                            v_fov = (-24.9, 2.0),
                            h_fov = (-45 , 45),
                            d_range = (0,100),
                            y_fudge=3,
                            x_fudge=0,
                            ):

    # Projecting to 2D
        x_points = points[:, 0]
    	y_points = points[:, 1]
    	z_points = points[:, 2]
    	r_points = points[:, 3]
    	d_points = np.sqrt(x_points ** 2 + y_points ** 2)  # map distance relative to origin
    	m_ranges = np.sqrt(x_points**2 + y_points**2 + z_points**2) # abs distance

    # We use map distance, because otherwise it would not project onto a cylinder,
    # instead, it would map onto a segment of slice of a sphere.

    # RESOLUTION AND FIELD OF VIEW SETTINGS
    	v_fov_total = -v_fov[0] + v_fov[1]
    	h_fov_total = -h_fov[0] + h_fov[1]
    # CONVERT TO RADIANS
    	v_res_rad = v_res * (np.pi / 180)
    	h_res_rad = h_res * (np.pi / 180)

    # MAPPING TO CYLINDER
    	x_img = np.arctan2(y_points, x_points) / h_res_rad
    	y_img = -(np.arctan2(z_points, d_points) / v_res_rad)

    # THEORETICAL MAX HEIGHT FOR IMAGE
    	d_plane = (v_fov_total/v_res) / (v_fov_total* (np.pi / 180))
    	h_below = d_plane * np.tan(-v_fov[0]* (np.pi / 180))
    	h_above = d_plane * np.tan(v_fov[1] * (np.pi / 180))
    	y_max = int(np.ceil(h_below+h_above + y_fudge))

#     d1_plane = (h_fov_total/h_res) / (h_fov_total* (np.pi / 180))
#     h_below = d1_plane * np.tan(-h_fov[0]* (np.pi / 180))
#     h_above = d1_plane * np.tan(h_fov[1] * (np.pi / 180))
#     x_max = int(np.ceil(h_below+h_above + x_fudge))
    
    # SHIFT COORDINATES TO MAKE 0,0 THE MINIMUM
    	x_min = -90 / h_res / 2
    	x_img = np.trunc(-x_img - x_min).astype(np.int32)
    	x_max = int(np.ceil(90.0 / h_res))

    	y_min = -((v_fov[1] / v_res) + y_fudge)
    	y_img = np.trunc(y_img - y_min).astype(np.int32)

    # CLIP DISTANCES
    	d_points = np.clip(d_points, a_min=d_range[0], a_max=d_range[1])

    # CONVERT TO IMAGE ARRAY
    	img = np.zeros([y_max + 1, x_max + 1,5], dtype=np.float)
    	img[y_img, x_img,0] = points[:, 0]
    	img[y_img, x_img,1]= points[:, 1]
    	img[y_img, x_img,2] =points[:,2] #scale_to_255(points[:,2]+5, min=0, max=10);
    	img[y_img, x_img,3] = points[:,3]#scale_to_255(points[:,3], min=0, max=1);
        img[y_img, x_img,4] = m_ranges;#d_points#scale_to_255(d_points, min=d_range[0], max=d_range[1])

        return img
    def scale_to_255(self,a, min, max, dtype=np.uint8):
#    """ Scales an array of values from specified min, max range to 0-255
#        Optionally specify the data type of the output (default is uint8)
#    """
        return (((a - min) / float(max - min)) * 255).astype(dtype)

    def calldata(self,cloud):

        clock = Clock()

	inbegin=datetime.datetime.now()

	assert isinstance(cloud, PointCloud2)
    	data = list(pc2.read_points(cloud, skip_nans=True, field_names = ("x", "y", "z","intensity")))
#	data =pc2.read_points(cloud, skip_nans=True)

	inend= datetime.datetime.now()
	print (inend-inbegin)

	transto5Dbegin = datetime.datetime.now()
	interlist = np.ones([1,4])
    #单帧点云每个点修改为5维
    	for ppoint in data:  
            if ppoint[0]>0:   #只要前方点
            	#data[kk+2]=data[kk+2]+3.2   #z轴全部变为正值
            	if ppoint[2]>=-5 and ppoint[2]<8:    #限制z轴范围，排除杂点
                   # r=(ppoint[0]**2+ppoint[1]**2+ppoint[2]**2)**0.5
                   # d=(ppoint[0]**2+ppoint[1]**2)**0.5
                   # seta = math.atan2(ppoint[2],d)*180.0/3.14
                    fi= (math.atan2(ppoint[1],ppoint[0])*180/3.14)
                    if abs(fi)<45:
                    #if seta+22>0:
                       # interlist=np.append (interlist,[[ppoint[0],ppoint[1],ppoint[2],ppoint[3],r,seta+22,fi+45]],axis=0)              
			interlist=np.append (interlist,[[ppoint[0],ppoint[1],ppoint[2],ppoint[3]]],axis=0)

    #删除interlist初始化的第一维
        interlist=np.delete(interlist,0,axis=0)
	transto5Dend = datetime.datetime.now()
	print (transto5Dend-transto5Dbegin)

	trans2front_begin = datetime.datetime.now()
    #将点以64×512数组形式储存
    	lidar=self.point_cloud_to_panorama(interlist,v_res=0.4,h_res = 0.16,\
                              v_fov = (-23.7, 3.5),h_fov = (-45, 45),d_range = (0,100),y_fudge=5,x_fudge=5)
    #裁剪图片成为64×512
    	for di in range(0,len(lidar)-64):
            lidar=np.delete(lidar,-1,axis=0)
    	h_reduce=len(lidar[1,:])-512;
    	for ddi in range(0,h_reduce/2):
            lidar=np.delete(lidar,-1,axis=1)
            lidar=np.delete(lidar,0,axis=1)	
	trans2front_end = datetime.datetime.now()
	print (trans2front_end-trans2front_begin)
# to perform prediction
        lidar_mask = np.reshape(
            (lidar[:, :, 4] > 0),
            [self._mc.ZENITH_LEVEL, self._mc.AZIMUTH_LEVEL, 1]
        )

        norm_lidar = (lidar - self._mc.INPUT_MEAN) / self._mc.INPUT_STD

        pred_cls = self._session.run(
            self._model.pred_cls,
            feed_dict={
                self._model.lidar_input: [norm_lidar],
                self._model.keep_prob: 1.0,
                self._model.lidar_mask: [lidar_mask]
            }
        )
        label = pred_cls[0]

        # point cloud for SqueezeSeg segments
        x = lidar[:, :, 0].reshape(-1)
        y = lidar[:, :, 1].reshape(-1)
        z = lidar[:, :, 2].reshape(-1)
        i = lidar[:, :, 3].reshape(-1)
        label = label.reshape(-1)
        cloud = np.stack((x, y, z, i, label))

        header = Header()
        header.stamp = rospy.Time().now()
        header.frame_id = "velodyne_link"

        # point cloud segments
        msg_segment = self.create_cloud_xyzil32(header, cloud.T)

        # publish
        self._pub.publish(msg_segment)
        rospy.loginfo("Point cloud processed. Took %.6f ms.",
                      clock.takeRealTime())



    def __init__(self, pub_topic, FLAGS, npy_path="", npy_file_list=""):

        os.environ['CUDA_VISIBLE_DEVICES'] = FLAGS.gpu
        self._mc = kitti_squeezeSeg_config()
        self._mc.LOAD_PRETRAINED_MODEL = False

        self._mc.BATCH_SIZE = 1
        self._model = SqueezeSeg(self._mc)
        self._saver = tf.train.Saver(self._model.model_params)

        self._session = tf.Session(
            config=tf.ConfigProto(allow_soft_placement=True))
        self._saver.restore(self._session, FLAGS.checkpoint)

         # Subscriber
	self._sub = rospy.Subscriber("/kitti_player/hdl64e",PointCloud2,self.calldata)	
	 # ed: Publisher
        self._pub = rospy.Publisher(pub_topic, PointCloud2, queue_size=1)

#        self.get_npy_from_lidar_2d(npy_path, npy_file_list)

#        self.idx = 0
 #       while not rospy.is_shutdown():
 #           self.prediction_publish(self.idx)
 #           self.idx += 1
 #           if self.idx > self.len_files:
 #               self.idx = 0

        rospy.spin()

    # Read all .npy data from lidar_2d folder
    def get_npy_from_lidar_2d(self, npy_path, npy_file_list):
        self.npy_path = npy_path
        self.npy_file_list = open(npy_file_list, 'r').read().split('\n')
        self.npy_files = []

        for i in range(len(self.npy_file_list)):
            self.npy_files.append(
                self.npy_path + self.npy_file_list[i] + '.npy')

        self.len_files = len(self.npy_files)
    

    # create pc2_msg with 5 fields
    def create_cloud_xyzil32(self, header, points):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1),
                  PointField('label', 16, PointField.FLOAT32, 1)]
        return pc2.create_cloud(header, fields, points)


if __name__ == '__main__':
    rospy.init_node('squeezeseg_ros_node')

    npy_path = rospy.get_param('npy_path')
    npy_file_list = rospy.get_param('npy_file_list')

    pub_topic = rospy.get_param('pub_topic')
    checkpoint = rospy.get_param('checkpoint')
    gpu = rospy.get_param('gpu')

    FLAGS = tf.app.flags.FLAGS
    tf.app.flags.DEFINE_string(
        'checkpoint', checkpoint,
        """Path to the model paramter file.""")
    tf.app.flags.DEFINE_string('gpu', gpu, """gpu id.""")

    npy_tensorflow_to_ros = NPY_TENSORFLOW_TO_ROS(pub_topic=pub_topic,
                                                  FLAGS=FLAGS,
                                                  npy_path=npy_path,
                                                  npy_file_list=npy_file_list)
