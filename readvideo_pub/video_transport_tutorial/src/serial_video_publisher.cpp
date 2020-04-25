#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  cv::VideoCapture cap(0);
  
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640); // 1600x1200,960x720,640x480,320x240
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480); 

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) 
  {
      ROS_INFO("can not opencv video device\n");
      return 1;
  }
   

  ros::Rate loop_rate(60);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
