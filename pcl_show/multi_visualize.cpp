//
// Created by lfg on 19-9-8.
//
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include<stdio.h>
#include<stdlib.h>
#include <string>
namespace fs = boost::filesystem;
int main(){
    std::string root_path = "./clusters_000000";
    fs::path path(root_path);
    //读取文件数量
    int num_pcd=0;
    fs::directory_iterator end_iter;
    for (fs::directory_iterator iter(path); iter!=end_iter; ++iter){
        num_pcd++;
    }
    std::cout<<num_pcd<<std::endl;
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addCoordinateSystem(0.01);
    viewer.initCameraParameters();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i < num_pcd;i++){
        //num 6位
        std::string num = std::to_string(i);
        for(int cc=0;cc<6;cc++){
            if (num.size() < 6) {
                num = "0" + num;
            }else{
                break;
            }
        }
        pcl::io::loadPCDFile (root_path+ "/cloud_"+ num +".pcd", *cloud);
        std::cout<<num<<std::endl;
        //随机设置颜色
        int r=  rand();
        int g=  rand();
        int b=  rand();
        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> theroyCircleCloud_color(cloud,r,g,b);
        //点云颜色渲染
        viewer.addPointCloud(cloud, theroyCircleCloud_color, "divider-cloud"+std::to_string(i));
        //设置点云大小
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "divider-cloud"+std::to_string(i));
        cloud->clear();
    }

//    viewer.resetCamera();
    while (!viewer.wasStopped()){
        viewer.spinOnce (100);
    }
    return 0;
}
