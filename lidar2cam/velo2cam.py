import csv
import numpy as np
import cv2
import os
import matplotlib as plt
from matplotlib import cm
#read calib
def read_calibration(calib_dir, pitch,yaw):
    data_file = open(calib_dir , 'r')
    data_reader = csv.reader(data_file, delimiter=' ')
    data = []

    for row in data_reader:
        data.append(row)

    data_file.close()

    innner_metic = data[0]
    innner_metic = innner_metic[1:]
    innner_metic = [float(innner_metic[i]) for i in range(9)]
    innner_metic = np.reshape(innner_metic, (3, 3))

    outer_metic = data[1]
    outer_metic = outer_metic[1:]
    outer_metic = [float(outer_metic[i]) for i in range(12)]
    outer_metic = np.reshape(outer_metic, (3, 4))
    pitch = pitch/180*np.pi
    yaw = yaw / 180 * np.pi
    pitch_rect = [[1,0,0],[0,np.cos(pitch),np.sin(pitch)],[0,-np.sin(pitch),np.cos(pitch)]]
    yaw_rext = [[np.cos(yaw),0,-np.sin(yaw)],[0,1,0],[np.sin(yaw),0,np.cos(yaw)]]
    outer_metic_rect = np.matmul(pitch_rect,outer_metic)
    outer_metic_rect = np.matmul(yaw_rext, outer_metic_rect)
    return np.matmul(innner_metic,outer_metic_rect)

#read lidar
def read_lidar(velo_dir):
    if os.path.exists(velo_dir):
        with open(velo_dir, 'rb') as fid:
            data_array = np.fromfile(fid, np.single)

        xyzi = data_array.reshape(-1, 4)

        x = xyzi[:, 0]
        y = xyzi[:, 1]
        z = xyzi[:, 2]
        i = xyzi[:, 3]

        return x, y, z, i
    else:
        return []
#draw point
def draw_point(image, points,depth):
    radius = 1
    thickness = -1
    cmap = cm.jet
    for i,point in enumerate(points):
        x = int(np.ceil(point[0]))
        y = int(np.ceil(point[1]))
        if x<= image.shape[1] and x >= 0 and y <= image.shape[0] and y >= 0:
            # print(x,y)
            if depth[i] > 0 :
                cols = depth[i]/100
                r = cm.jet(cols)[0]*255
                g = cm.jet(cols)[1]*255
                b = cm.jet(cols)[2]*255
                # print('cols',cols,'r',r,'g',g,'b',b)
                cv2.circle(image, (x,y),radius,(r,g,b), thickness)
    return image

pitch_rect = 4 #du
yaw_rect = -1.55
project_metic = read_calibration('./test/calib.txt', pitch_rect,yaw_rect)
x, y, z, i = read_lidar('./test/000000.bin')
one = np.ones_like(x)
points = np.stack([x, y, z, one],axis=0)

velo2img = np.matmul(project_metic,points)
velo2img = velo2img/velo2img[2]
velo2img = np.stack(velo2img,axis=1)
image = cv2.imread('./test/000000.png')
image_point = draw_point(image,velo2img,points[0])

cv2.namedWindow("velo2img")
cv2.imshow('velo2img', image_point)
cv2.waitKey () # 显示 10000 ms 即 10s 后消失
cv2.destroyAllWindows()
