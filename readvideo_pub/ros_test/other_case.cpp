#include <iostream>
//ROS
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
//opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// fusion_msgs
#include <ros_test/BboxLes.h>
#include <ros_test/BboxL.h>
#include <iostream>

#include <stdio.h>
#include <sstream>
#include <vector>
class OpencvHandler
{
 public:
    OpencvHandler():it(nh)
    {
     //image_transport::ImageTransport it(nh);
     sub1= it.subscribe("/kitti_player/color/left/image_rect", 1, &OpencvHandler::color_pic, this);
     sub2= nh.subscribe("ROI", 10, &OpencvHandler::ROIcb, this);
     pub1= it.advertise("ROIpicture", 1);
     pub2= nh.advertise<opencv_deal::BboxLes>("ROIL2D", 1);   
     }

     cv::Mat dstImage;
     cv_bridge::CvImagePtr cv_ptr;
	
    void color_pic(const sensor_msgs::ImageConstPtr& msg)
    {
        try{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
	}
        catch (cv_bridge::Exception& e)  
        {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
         return;  
        }
	dstImage= cv_ptr->image;

    }

     void ROIcb (const sensor_msgs::PointCloud2& input)
    {
     // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
      pcl::PointCloud<pcl::PointXYZRGB> Rec;
      pcl::fromROSMsg (input, Rec);   //关键的一句数据的转换
      cv::Mat projectionMatrix,cameraMatrix,velo2camera;
      std::string calibFilePath="/home/lfg/my_work/graduate/src/opencv_deal/src/calib.txt"; 

      readOdometryCalib(calibFilePath,projectionMatrix,cameraMatrix,velo2camera);


      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dstImage).toImageMsg();

     // pub1.publish(cv_ptr->toImageMsg());
      pub1.publish(msg);
    }
 

    protected:
        ros::NodeHandle nh;
	ros::Subscriber sub2;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub1;
        image_transport::Publisher pub1;
	ros::Publisher pub2;
        struct Bbox{
        float minx_bb =0;
        float maxx_bb =0;
        float miny_bb =0;
        float maxy_bb =0;
        int flag_bb =0;
	float centerx;
	float centery;
        };
	pcl::PointCloud<pcl::PointXYZRGB> lastRec;	
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "showROI");

  OpencvHandler handler;
  // Spin
  ros::spin ();

  return 0;
}

