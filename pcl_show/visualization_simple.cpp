//
// Created by lfg on 19-8-12.
//

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("./cloud_000000.pcd", *cloud);
//    pcl::io::loadPCDFile ("/home/lfg/my_work/deecamp/my_work/pcl_show/clusters_000000/cloud_000070.pcd", *cloud);
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
//    viewer.resetCamera();
    while (!viewer.wasStopped ())
    {

    }
    return 0;
}
