//
// Created by lfg on 19-8-12.
//
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <iostream>
#include <string>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace std;
using namespace pcl;

void PCLpoint(Mat DisparityMap, Mat imgor)
{
    PointCloud<PointXYZRGB> cloud_a;
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    float rows = DisparityMap.rows;
    float cols = DisparityMap.cols;

    cloud_a.height = rows;
    cloud_a.width = cols;
    cloud_a.points.resize(cloud_a.width * cloud_a.height);


    int height = 0;
    for (int i = 0; i < rows; i++)
    {
        uchar *pdis = DisparityMap.ptr<uchar>(i);
        uchar *pimg = imgor.ptr<uchar>(i);
        for (int j = 0; j < cols * 3; j = j + 3)
        {
            unsigned int num = i*cols + j / 3;
            float fx = 1261.5;
            float fy = 1147.04;
            float B = 0.54;
            float cx = 976.24;
            float cy = 530.59;
            float Zw, Xw, Yw;

            if (pdis[j / 3]>0){
                Zw = fx /pdis[j / 3];
//                cout<< "Zw"<<Zw<<endl;
                Yw = ( i - cy)* Zw / fy;
                Xw = ( j / 3 - cx)* Zw / fx;
            }
            else{
                Zw = 0; Yw = 0; Xw = 0;
            }
            cout<<"Yw"<<Yw<<endl;
            if(Yw<-70){
                Zw = 0; Yw = 0; Xw = 0;
            }

            cloud_a.points[num].b = pimg[j];
            cloud_a.points[num].g = pimg[j + 1];
            cloud_a.points[num].r = pimg[j + 2];

            cloud_a.points[num].x = Xw;
            cloud_a.points[num].y = Yw;
            cloud_a.points[num].z = Zw;
        }
    }
    *cloud = cloud_a;
    visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){
        int user_data = 9;
    }
}

void viewerOneOff(visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
}


int main(int argc, char** argv)
{
    Mat rgbsrc = imread("./1.jpg");
    Mat depthsrc = imread("./d1.png",2);
    if (rgbsrc.empty() || depthsrc.empty()) {
        printf("no image read");
        return -1;
    }
    cout << "type:" << depthsrc.type() << endl;
    cout << "channels:" << depthsrc.channels() << endl;
//    cout << "r(C语言风格_openCV3) = " << format(depthsrc, Formatter::FMT_C) << ";" << endl;
//    imshow("depthsrc",depthsrc);
    PCLpoint(depthsrc, rgbsrc);
    waitKey();
}
