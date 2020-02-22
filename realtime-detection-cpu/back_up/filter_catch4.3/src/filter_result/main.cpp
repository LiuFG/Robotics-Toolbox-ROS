/*=================================================================
 * Calculate Background Model of a list of Frames(Normally a video stream) in the
 * method of Background Difference Method & OTSU Algorithm By OpenCV.
 *
 * Copyright (C) 2017 Chandler Geng. All rights reserved.
 *
 *     This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *     You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place, Suite 330, Boston, MA 02111-1307 USA
===================================================================
*/

/*=================================================
 * Version:
 * v1.0: 原版程序由IplImage转换为Mat；
 * v1.1: 背景差分法封装成类: BGDiff；
 * v1.2: 补充注释；
 * v1.3: 该方法与高斯混合背景模型不同，命名有误，改为背景差分法；
===================================================
*/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "classify.h"
#include <iostream>
#include <queue>
#include <stdlib.h>
#include "harris.h"
#include <opencv2/xfeatures2d.hpp>

using namespace cv;



class pointdeal
{
public:
	pointdeal()
	{
//		pub1 = nh.advertise<sensor_msgs::PointCloud2> ("ROI", 1000);
		sub = nh.subscribe("back_camera/image_raw", 1, &pointdeal::chatterCallback,this);
	    time=0;
	}

	void search_points(int grid_row,int grid_col,int label)
	{
		ROS_INFO("step into Collect ingrid points");
		bool exist_flag=false;
		int vec_num=0;

		auto count = labelpoints.size();
		for (int i = 0; i < count;i++)
		{
			if(labelpoints[i].label == label )
			{
				exist_flag=true;
				vec_num=i;
				break;
			}
		}

		if(exist_flag)
		{
			point2D memp;
			for(int k=0;k<grid[0];k++)
			{
				for(int i=0;i<grid[1];i++)
				{
					if(result_filter.at<int>(grid_row*grid[0]+k,grid_col*grid[1]+i)>0)
					{
						memp.x=grid_row*grid[0]+k;
						memp.y=grid_col*grid[1]+i;
						//更新类
						labelpoints[vec_num].vertex_min.x=min(memp.x,labelpoints[vec_num].vertex_min.x);
						labelpoints[vec_num].vertex_min.y=min(memp.y,labelpoints[vec_num].vertex_min.y);
						labelpoints[vec_num].vertex_max.x=max(memp.x,labelpoints[vec_num].vertex_max.x);
						labelpoints[vec_num].vertex_max.y=max(memp.y,labelpoints[vec_num].vertex_max.y);
						labelpoints[vec_num].points.push_back(memp);
					}
				}
			}
		}
		else
		{
			class_label grid_points;
			point2D memp;
			for(int k=0;k<grid[0];k++)
			{
				for(int i=0;i<grid[1];i++)
				{
					if(result_filter.at<int>(grid_row*grid[0]+k,grid_col*grid[1]+i)>0)
					{
						memp.x=grid_row*grid[0]+k;
						memp.y=grid_col*grid[1]+i;
						grid_points.vertex_min.x=min(memp.x,grid_points.vertex_min.x);
						grid_points.vertex_min.y=min(memp.y,grid_points.vertex_min.y);
						grid_points.vertex_max.x=max(memp.x,grid_points.vertex_max.x);
						grid_points.vertex_max.y=max(memp.y,grid_points.vertex_max.y);
						grid_points.points.push_back(memp);
					}
				}
			}
			grid_points.label=label;
			labelpoints.push_back(grid_points);
		}



	}

	bool if_occupy( int grid_row,int grid_col)
	{
		int flag=0;
		for(int k=0;k<grid[0];k++)
		{
			for(int i=0;i<grid[1];i++)
			{
				flag=result_filter.at<int>(grid_row*grid[0]+k,grid_col*grid[1]+i);
				//std::cout<<"filter ingrid value:"<<flag<<std::endl;
				if(flag>0)
				{
					ROS_INFO("it is occupied");
					return true;
				}

			}
		}
		return false;
	}

	void BFS_my(int grid_row, int grid_col, int Label_num)
	{
		ROS_INFO("step into BFS");
		if (grid_row== 0 || grid_row>label_image.rows-2||grid_col== 0||grid_col>label_image.cols-2)
		{
			//种子点在边缘
		}
		else
		{
			std::queue<Point> grid_list;
			grid_list.push(Point(grid_row, grid_col));//行列记点
			Search_state.at<unsigned char>(grid_row, grid_col) = 255;
            label_image.at<int>(grid_row, grid_col) = Label_num;
			while (!grid_list.empty())
			{
				Point u = grid_list.front();
				grid_list.pop();

				for (int n = grid_row - 1; n <= grid_row + 1; n++)//°ËÁÚÓò
				{
					for (int m = grid_col - 1; m <= grid_col + 1; m++)
					{
						if (m == grid_col && n == grid_row) {}    //中心点不执行
						else if (!if_occupy(n,m)) {
							Search_state.at<unsigned char>(n, m) = 255;
						} //邻域点为空
						else if (Search_state.at<unsigned char>(n, m) >0) {} //ÒÑËÑË÷
						else
						{
							label_image.at<int>(n, m) = Label_num;
							grid_list.push(Point(n,m));
							Search_state.at<unsigned char>(n, m) = 255; //Ö»ÒªÑ¹œø¶ÓÁÐ£¬ŸÍÒÑËÑË÷

							search_points(n,m,Label_num);
						}
					}
				}
			}
		}


	}

	void drawbox()
	{
		ROS_INFO("step into Drawbox");
		if(!labelpoints.empty())
		{
			for(auto iter=labelpoints.begin();iter<labelpoints.end();iter++)
			{
				//int weight = iter->vertex_max.y-iter->vertex_min.y;
				//int height = iter->vertex_max.x-iter->vertex_min.x;
				rectangle(srcImage,cvPoint(iter->vertex_min.y,iter->vertex_min.x),cvPoint(iter->vertex_max.y,iter->vertex_max.x),Scalar(0,0,255),1,1,0);
				rectangle(result_filter,cvPoint(iter->vertex_min.y,iter->vertex_min.x),cvPoint(iter->vertex_max.y,iter->vertex_max.x),Scalar(255,255,255),1,1,0);

			}
		}
		else
			ROS_INFO("labelpoints are empty");


		//FPS
		time=((double)getTickCount()-time)/getTickFrequency();
		double fps =1.0/time;
		char num_fps[10];
		sprintf(num_fps,"%.2f",fps);
		std::string sfps("FPS:");
		sfps+=num_fps;
		putText(srcImage,sfps,Point(5,20),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));

		//show
		imshow("boudingbox",srcImage);
		imshow("binary_image",result_filter);
		imshow("search status",Search_state);
		labelpoints.clear();
		waitKey(1);
	}

	void BFS_seg()
    {
        int Label_value = 0;

        for(int k=0;k<label_image.rows;k++)
        {
            for(int i=0;i<label_image.cols;i++)
            {
                int Label = label_image.at<unsigned char>(k, i);
                int search_status=Search_state.at<unsigned char>(k, i);
                //std::cout<<"labelmap"<<Label<<"search_status_map"<<search_status<<std::endl;
                if(Label == 0 && search_status==0)
                {
                    bool value = if_occupy(k,i);
                    //std::cout<<"grid value"<<value<<std::endl;
                    if(value)
                    {
                        Label_value++;
                        BFS_my(k, i, Label_value);
                    }
                    else
                    {
                        Search_state.at<unsigned char>(k, i)=255;
                    }
                }
            }
        }
        drawbox();
    }

    void _harris()
    {
        Mat corner_srcImage,grayImage;
        cvtColor(srcImage,grayImage,CV_BGR2GRAY);
        corner_srcImage=grayImage.clone();
        //==============harris================
        harris Harris;
        // 计算角点
        Harris.detect(corner_srcImage);
        //获得角点
        std::vector<cv::Point> pts;
        Harris.getCorners(pts,0.01);
        // 标记角点
        Harris.drawOnImage(corner_srcImage,pts);
        cv::namedWindow ("harris");
        cv::imshow ("harris",corner_srcImage);
        waitKey(1);

    }
    void _shiTomasi()
    {
        Mat cornerTomasi_result,grayImage;
        cvtColor(srcImage,grayImage,CV_BGR2GRAY);
        cornerTomasi_result=grayImage.clone();
        //=============shi-Tomasi=============
        std::vector<Point> cornersTomasi;
        goodFeaturesToTrack(srcImage,cornersTomasi,200,0.01,10);
        harris().drawOnImage(cornerTomasi_result,cornersTomasi);
        cv::namedWindow ("shi-Tomasi");
        cv::imshow ("shi-Tomasi",cornerTomasi_result);
        waitKey(1);

    }
//    void _FAST()
//    {
//        Mat grayImage,fast_srcImage;
//        cvtColor(srcImage,grayImage,CV_BGR2GRAY);
//        fast_srcImage=grayImage.clone();
//        //=======FAST============
//        std::vector<cv::KeyPoint> fast_keypoints;
//        cv::FastFeatureDetector fast(40,true);
//        fast.detect (fast_srcImage,fast_keypoints);
//        cv::drawKeypoints (fast_srcImage,fast_keypoints,fast_srcImage,cv::Scalar::all(255),cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
//        cv::namedWindow ("FAST");
//        cv::imshow ("FAST",fast_srcImage);
//        waitKey(1);
//
//    }
    void _SIFT()
    {
        Mat grayImage,sift_srcImage;
        cvtColor(srcImage,grayImage,CV_BGR2GRAY);
        sift_srcImage=grayImage.clone();
        //========SIFT===============
        std::vector<KeyPoint> sift_keypoints;
        cv::xfeatures2d::SiftFeatureDetector sift;
        sift.create(50);
        sift.detect(sift_srcImage,sift_keypoints);
        drawKeypoints(sift_srcImage,sift_keypoints,sift_srcImage,Scalar(255,255,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow ("SIFT");
        imshow ("SIFT",sift_srcImage);
        waitKey(1);
    }
//    void _SURF()
//    {
//        Mat grayImage,surf_srcImage;
//        cvtColor(srcImage,grayImage,CV_BGR2GRAY);
//        surf_srcImage=grayImage.clone();
//        //============SUFT=========
//        std::vector<KeyPoint> surf_keypoints;
//        cv::xfeatures2d::SurfFeatureDetector surf(2500);
//        surf.detect(surf_srcImage,surf_keypoints);
//        drawKeypoints(surf_srcImage,surf_keypoints,surf_srcImage,Scalar(255,255,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//        cv::namedWindow ("SURF");
//        imshow ("SURF",surf_srcImage);
//        waitKey(1);
//    }
    void corner_detect()
    {

    }

	void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
	{
	    time=(double)getTickCount();

		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		Mat sgrayImage,hsvImage;
		srcImage=cv_ptr->image;
		label_image=Mat::zeros(srcImage.rows/grid[0],srcImage.cols/grid[1], CV_8UC1);
		Search_state=Mat::zeros(label_image.rows,label_image.cols, CV_8UC1);
		cvtColor(srcImage,sgrayImage,CV_BGR2GRAY);
		cvtColor(srcImage, hsvImage, CV_BGR2HSV);


		float IH[] = { -0.125,0.0,0.125,-0.25,0.0,0.25,-0.125,0.0,0.125 };
		float HI[] = { -0.125,-0.25,-0.125,0.0,0.0,0.0,0.125,0.25,0.125 };
		float HH[] = { 0.25,0.0,-0.25,0.0,0.0,0.0,-0.25,0.0,0.25 };
		Mat matHI, matHH, matIH;
		Mat rHI, rHH, rIH;

		Mat KerHI( 3, 3, CV_32FC1, HI);
		Mat KerIH(3, 3, CV_32FC1, IH);
		Mat KerHH( 3, 3, CV_32FC1, HH);
		filter2D(sgrayImage,matHI, CV_32FC1,KerHI);
		filter2D(sgrayImage, matIH, CV_32FC1, KerIH);
		filter2D(sgrayImage, matHH, CV_32FC1, KerHH);

		threshold(matHI, rHI, 3, 255, CV_THRESH_BINARY);
		threshold(matIH, rIH, 3, 255, CV_THRESH_BINARY);
		threshold(matHH, rHH, 3, 255, CV_THRESH_BINARY);

		
		result_filter=rHH.clone();
//		imshow("HI", rHI);
//		imshow("IH", rIH);
//		imshow("HH", rHH);
		imshow("result_filter",result_filter);
	    waitKey(1);

        //BFS_seg();

        _SIFT();

	}

protected:
	ros::NodeHandle nh;
	//ros::Publisher pub1,pub2;
	ros::Subscriber sub;
	std::vector<class_label> labelpoints;
	Mat label_image;
	Mat Search_state;
	Mat srcImage;
	int grid[2]={64,64};
	Mat result_filter;
	double time;
};
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "filtercatch");

	pointdeal handler;

	// Spin
	ros::spin ();
	return 0;
}