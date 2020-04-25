#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <camera_info_manager/camera_info_manager.h>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  sensor_msgs::CameraInfo cam_info;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  string path = "./src/data/";
  path = path + "blue.mp4";//用户自己添加视频文件名字
 
  VideoCapture cap(path);//open video from the path
 
//  cap.set(CV_CAP_PROP_FRAME_WIDTH,640); // 1600x1200,960x720,640x480,320x240
//  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480); 

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 100);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info",100);
  
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) 
  {
      ROS_INFO("can not opencv video device\n");
      return 1;
  }
   
  bool isSuccess = true;
  ros::Rate loop_rate(10);
  while (nh.ok()) {
    cap >> frame;
    //===========编写自己的代码

    //========================
    isSuccess = cap.read(frame);
    if(!isSuccess)//if the video ends, then break
    {
    std::cout<<"video ends"<<std::endl;
    break;
    }
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {

    cam_info.height=640;
    cam_info.width=320;

    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info.K = boost::assign::list_of(1.0) (0.0) (frame.cols/2.0)//(img->width/2.0)
            (0.0) (1.0) (frame.rows/2.0)
            (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info.R = boost::assign::list_of (1.0) (0.0) (0.0)
            (0.0) (1.0) (0.0)
            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info.P = boost::assign::list_of (1.0) (0.0) (frame.cols/2.0) (0.0)
            (0.0) (1.0) (frame.rows/2.0) (0.0)
            (0.0) (0.0) (1.0) (0.0);


      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      pub2.publish(cam_info);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
    return 0;
}
