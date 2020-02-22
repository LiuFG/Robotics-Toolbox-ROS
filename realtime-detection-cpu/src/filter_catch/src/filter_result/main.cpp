/*=================================================
 * Editor:
===================================================
*/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "classify.h"
#include <queue>
#include <stdlib.h>
#include "harris.h"
//#include "key_points.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>

using namespace std;
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

using namespace cv;

class pointdeal
{
public:
	pointdeal()
	{
//		pub1 = nh.advertise<sensor_msgs::PointCloud2> ("ROI", 1000);
		sub = nh.subscribe("/camera/color/image_raw", 1, &pointdeal::colorCallback,this);
        sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &pointdeal::depthCallback,this);
		//back_camera/image_raw
		time=0;
	}

	void search_points(int grid_row,int grid_col,int label)
	{
		//ROS_INFO("step into Collect ingrid points");
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
        int lastvalue;
        ushort lastdepth;
		if(exist_flag)
		{
			point2D memp;

			for(int k=0;k<grid[0];k++)
			{
				for(int i=0;i<grid[1];i++)
				{
					if(result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i)>0)
					{
						memp.x=grid_row*grid[0]+k;
						memp.y=grid_col*grid[1]+i;
						//更新类
                        lastvalue=labelpoints[vec_num].vertex_min.x;
						labelpoints[vec_num].vertex_min.x=min(memp.x,lastvalue);
                        lastvalue=labelpoints[vec_num].vertex_min.y;
						labelpoints[vec_num].vertex_min.y=min(memp.y,lastvalue);
                        lastvalue=labelpoints[vec_num].vertex_max.x;
						labelpoints[vec_num].vertex_max.x=max(memp.x,lastvalue);
                        lastvalue=labelpoints[vec_num].vertex_max.y;
						labelpoints[vec_num].vertex_max.y=max(memp.y,lastvalue);
						//TODO 加速：不压点
						//labelpoints[vec_num].points.push_back(memp);
						labelpoints[vec_num].point_num++;
						if(depthImage.at<ushort>(memp.x,memp.y)>0)
						{
                            lastdepth=labelpoints[vec_num].depth_value;
							labelpoints[vec_num].depth_value=min(depthImage.at<ushort>(memp.x,memp.y),lastdepth);
						}
					}
				}
			}
			//update
            labelpoints[vec_num].centerpoint.x=(labelpoints[vec_num].vertex_min.x+labelpoints[vec_num].vertex_max.x)/2;
            labelpoints[vec_num].centerpoint.y=(labelpoints[vec_num].vertex_min.y+labelpoints[vec_num].vertex_max.y)/2;
            labelpoints[vec_num].width=labelpoints[vec_num].vertex_max.y-labelpoints[vec_num].vertex_min.y;
            labelpoints[vec_num].height=labelpoints[vec_num].vertex_max.x-labelpoints[vec_num].vertex_min.x;

		}
		else
		{
			class_label grid_points;
			point2D memp;
			for(int k=0;k<grid[0];k++)
			{
				for(int i=0;i<grid[1];i++)
				{
					if(result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i)>0)
					{
						memp.x=grid_row*grid[0]+k;
						memp.y=grid_col*grid[1]+i;
                        lastvalue=grid_points.vertex_min.x;
						grid_points.vertex_min.x=min(memp.x,lastvalue);
                        lastvalue=grid_points.vertex_min.y;
						grid_points.vertex_min.y=min(memp.y,lastvalue);
                        lastvalue=grid_points.vertex_max.x;
						grid_points.vertex_max.x=max(memp.x,lastvalue);
                        lastvalue=grid_points.vertex_max.y;
						grid_points.vertex_max.y=max(memp.y,lastvalue);
						//TODO 加速：不压点
						//grid_points.points.push_back(memp);
						grid_points.point_num++;
						if(depthImage.at<ushort>(memp.x,memp.y)>0)
						{
                            lastdepth=grid_points.depth_value;
							grid_points.depth_value=min(depthImage.at<ushort>(memp.x,memp.y),lastdepth);
						}
					}
				}
			}
			//加center，w,h
            grid_points.centerpoint.x=(grid_points.vertex_min.x+grid_points.vertex_max.x)/2;
            grid_points.centerpoint.y=(grid_points.vertex_min.y+grid_points.vertex_max.y)/2;
            grid_points.width=grid_points.vertex_max.y-grid_points.vertex_min.y;
            grid_points.height=grid_points.vertex_max.x-grid_points.vertex_min.x;
            grid_points.label=label;
			labelpoints.push_back(grid_points);
		}



	}

    bool if_occupy( int grid_row,int grid_col)
    {
        int flag=0;
        int omergeflag=0;
        int flagnum_lu=0;
        int flagnum_ld=0;
        int flagnum_ru=0;
        int flagnum_rd=0;
        int judgenum=2;//max(int(floor(float(grid[0])/10)),1);
        //左上角
        for(int k=0;k<grid[0]/2;k++)
        {
            for(int i=0;i<grid[1]/2;i++)
            {
                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
                if(flag>0)
                    flagnum_lu++;
                if(flagnum_lu>judgenum)
                {
                    omergeflag++;
                    break;
                }
            }
        }
        //左下

        for(int k=grid[0]/2;k<grid[0];k++)
        {
            for(int i=0;i<grid[1]/2;i++)
            {
                flag = 0;
                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
                if(flag>0)
                    flagnum_ld++;
                if(flagnum_ld>judgenum)
                {
                    omergeflag++;
                    break;
                }
            }
        }
       // 右上

        for(int k=0;k<grid[0]/2;k++)
        {
            for(int i=grid[1]/2;i<grid[1];i++)
            {
                  flag = 0;
                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
                if(flag>0)
                    flagnum_ru++;
                if(flagnum_ru>judgenum)
                {
                    omergeflag++;
                    break;
                }
            }
        }
        //右下

        for(int k=grid[0]/2;k<grid[0];k++)
        {
            for(int i=grid[1]/2;i<grid[1];i++)
            {
                flag = 0;
                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
                if(flag>0)
                {
                    flagnum_rd++;
                }
                if(flagnum_rd>judgenum)
                {
                    omergeflag++;
                    break;
                }
            }
        }
/*===============================
//        for(int k=0;k<grid[0]/4;k++)
//        {
//            for(int i=0;i<grid[1]/4;i++)
//            {
//                flag=0;
//                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
//                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
//                if(flag>0)
//                    flagnum_rd++;
//                if(flagnum_rd>judgenum)
//                {
//                    omergeflag++;
//                    break;
//                }
//            }
//        }
//        for(int k=grid[0]/4;k<2*grid[0]/4;k++)
//        {
//            for(int i=grid[1]/4;i<2*grid[1]/4;i++)
//            {
//                flag=0;
//                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
//                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
//                if(flag>0)
//                    flagnum_lu++;
//                if(flagnum_lu>judgenum)
//                {
//                    omergeflag++;
//                    break;
//                }
//            }
//        }
//
//        for(int k=2*grid[0]/4;k<3*grid[0]/4;k++)
//        {
//            for(int i=2*grid[1]/4;i<3*grid[1]/4;i++)
//            {
//                flag=0;
//                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
//                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
//                if(flag>0)
//                    flagnum_ld++;
//                if(flagnum_ld>judgenum)
//                {
//                    omergeflag++;
//                    break;
//                }
//            }
//        }
//
//        for(int k=3*grid[0]/4;k<4*grid[0]/4;k++)
//        {
//            for(int i=3*grid[1]/4;i<4*grid[1]/4;i++)
//            {
//                flag=0;
//                flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
//                //std::cout<<"filter ingrid value:"<<flag<<std::endl;
//                if(flag>0)
//                    flagnum_ru++;
//                if(flagnum_ru>judgenum)
//                {
//                    omergeflag++;
//                    break;
//                }
//            }
//        }*/


        if(omergeflag>1)//(flagnum_rd>judgenum&&flagnum_ru>judgenum)||(flagnum_ld>judgenum&&flagnum_lu>judgenum))
        {
            //ROS_INFO("it is occupied");
            return true;
        }
        else
            return false;
    }

	void BFS_my(int grid_row, int grid_col, int Label_num)
	{
		//ROS_INFO("step into BFS");
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

	void merge_()
    {

        for(auto iter=labelpoints.begin();iter<labelpoints.end();iter++)
        {
			bool mflag=false;//是否已有点
			//if(iter->points.size()>50)  TODO 加速：不压点
			//TODO 参数 重要显示参数
			if(iter->height*iter->width>900)
			{
				if(mergepoints.empty())
				{
					mergepoints.push_back(*iter);
				}
				else{
					for(auto miter=mergepoints.begin();miter<mergepoints.end();miter++)
					{
						if(iter->vertex_min.x==miter->vertex_min.x&&iter->vertex_min.y==miter->vertex_min.y)
						{
							mflag=true;
							break;
						}
					}
					if(!mflag)
					{
						mergepoints.push_back(*iter);
					}
				}
			}

        }
		//存储box TODO 可以综合前几帧
		lastbox.clear();
		for(auto miter=mergepoints.begin();miter<mergepoints.end();miter++)
		{
			miter->prob=1;//-abs(float(miter->width/miter->height)-1);
			//TODO float错误
			lastbox.push_back(*miter);
		}

    }

    void recshow(point2D lupoint,point2D rdpoint,int height)
    {
        Mat medImage=srcImage.clone();
        rectangle(srcImage,cvPoint(lupoint.y,lupoint.x),cvPoint(rdpoint.y,rdpoint.x),Scalar(0,0,255),1,1,0);
        rectangle(result_filter,cvPoint(lupoint.y,lupoint.x),cvPoint(rdpoint.y,rdpoint.x),Scalar(255,255,255),1,1,0);
        cv::Point pt[2][4];
        pt[0][0] = Point(lupoint.y,lupoint.x);
        pt[0][1] = Point(rdpoint.y,lupoint.x);
        pt[0][2] = Point(rdpoint.y,lupoint.x+height/3);
        pt[0][3] = Point(lupoint.y,lupoint.x+height/3);
        pt[1][0] = Point(lupoint.y,lupoint.x+height/2);
        pt[1][1] = Point(rdpoint.y,lupoint.x+height/2);
        pt[1][2] = Point(rdpoint.y,lupoint.x+height);
        pt[1][3] = Point(lupoint.y,lupoint.x+height);
        const cv::Point* ppt0[1]={pt[0]};
        const cv::Point* ppt1[1]={pt[1]};
        fillConvexPoly(medImage,*ppt0,4,Scalar(255,0,0));
        fillConvexPoly(medImage,*ppt1,4,Scalar(0,255,0));
        addWeighted(srcImage,0.8,medImage,0.2,0,srcImage);
    }

	void drawbox()
	{

		//ROS_INFO("step into Drawbox");
		if(!labelpoints.empty())
		{
			if(labelpoints.size()>1)
			{
				collectneighbour();
			}
            merge_();  //提取不重复的框
            //TODO 直接换成labelpoints试试
			if(!mergepoints.empty())
            {
                for(auto iter=mergepoints.begin();iter<mergepoints.end();iter++)
                {
                	recshow(iter->vertex_min,iter->vertex_max,iter->height);
                }
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
		putText(srcImage,sfps,Point(5,30),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255));

        labelpoints.clear();
        mergepoints.clear();
		//show
		imshow("boudingbox",srcImage);
		imshow("binary_image",result_filter);
		imshow("search status",Search_state);
		waitKey(1);
	}

	bool if_seperate(point2D center1,point2D center2)
    {
        int beginx=min(center1.x,center2.x);
        int endx=max(center1.x,center2.x);
        int beginy=min(center1.y,center2.y);
        int endy=max(center1.y,center2.y);

        for(int k=beginx;k<endx;k++) {
            for (int i = beginy; i < endy; i++) {
                int depth_grad=abs(depthImage.at<ushort>(k,i+1)-depthImage.at<ushort>(k,i));
                //TODO 参数
                if(depth_grad>200)
                {
                    return true;
                }
            }
        }
        return false;
    }

	void _update()
    {
        for(auto iter=labelpoints.begin();iter!=labelpoints.end();iter++)
        {
            for(auto iiter=labelpoints.begin();iiter!=labelpoints.end();iiter++)
            {
            	//TODO 参数影响大
                if(iter->point_num>10&&iiter->point_num>10)
                {
					//if(iter->point_num<4030&&iiter->point_num<4030)
                        float dis_x=abs(iter->centerpoint.x-iiter->centerpoint.x);
                        float dis_y=abs(iter->centerpoint.y-iiter->centerpoint.y);

                        //TODO 参数影响大,box的距离判断
                        if(dis_x*2/(iter->height+iiter->height)<2&&dis_y*2/(iter->width+iiter->width)<2)
                        {
                            //TODO 合框不影响本车
                            //bool ifseperate=if_seperate(iter->centerpoint,iiter->centerpoint);
                            bool diff=abs(iter->depth_value-iiter->depth_value)>400;
                            if(diff){
                            }
                            //else if(0){}
                            else{
                                //更新labelpoints
                                iter->vertex_min.x=min(iter->vertex_min.x,iiter->vertex_min.x);
                                iiter->vertex_min.x=iter->vertex_min.x;
                                iter->vertex_min.y=min(iter->vertex_min.y,iiter->vertex_min.y);
                                iiter->vertex_min.y=iter->vertex_min.y;

                                iter->vertex_max.x=max(iter->vertex_max.x,iiter->vertex_max.x);
                                iiter->vertex_max.x=iter->vertex_max.x;
                                iter->vertex_max.y=max(iter->vertex_max.y,iiter->vertex_max.y);
                                iiter->vertex_max.y=iter->vertex_max.y;

                                iter->depth_value=min(iter->depth_value,iiter->depth_value);
                                iiter->depth_value=iter->depth_value;

                                //更新其他参数
                                iter->centerpoint.x=(iter->vertex_min.x+iter->vertex_max.x)/2;
                                iter->centerpoint.y=(iter->vertex_min.y+iter->vertex_max.y)/2;
                                iter->width=iter->vertex_max.y-iter->vertex_min.y;
                                iter->height=iter->vertex_max.x-iter->vertex_min.x;
                                iiter->centerpoint.x=iter->centerpoint.x;
                                iiter->centerpoint.y=iter->centerpoint.y;
                                iiter->width=iter->width;
                                iiter->height=iter->height;

                                iter->point_num=iter->point_num+iiter->point_num;
                                iiter->point_num=iter->point_num;
                            }
                        }

                }

            }
        }
    }

	void collectneighbour()
	{
        _update();
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

    void filter_reinforce(Mat &filterImage)
	{

//			Mat mask=Mat::zeros(result_filter.rows,result_filter.cols, CV_8UC1);
//			int nr=result_filter.rows;
//			int nc=result_filter.cols;
//			for(int i=0;i<nr;i++)
//			{
//				const uchar* inData=result_filter.ptr<uchar>(i);
//				for(int j=0;j<nc;j++)
//				{
//					if(i>)
//					*outData++=*inData++/div*div+div/2;
//					Mat mask(3, 3, CV_8UC1, Scalar(0));
//					src.setTo(100, mask);
//				}
//			}
//            Mat hsvImage;
//            cvtColor(srcImage, hsvImage, CV_BGR2HSV);
//            std::vector<Mat> channels;
//            Mat blueImage,greenImage,redImage;
//            Mat bblueImage,bgreenImage,bredImage;
//
//            split(srcImage,channels);
//            blueImage=channels.at(0);
//            greenImage=channels.at(1);
//            redImage=channels.at(2);
//            threshold(blueImage, bblueImage, 254, 255, CV_THRESH_BINARY);
//            threshold(greenImage, bgreenImage, 254, 255, CV_THRESH_BINARY);
//            threshold(redImage, bredImage, 254, 255, CV_THRESH_BINARY);


            for (int i = 0; i < filterImage.rows; i++)
            {
                for (int j = 0; j < filterImage.cols; j++)
                {
                    if(1)//bredImage.at<uchar>(i,j)>0)
                    {
                        filterImage.at<uchar>(i,j) = filterImage.at<uchar>(i,j)*filterImage.at<uchar>(i,j);
                    }
                }
            }

		imshow("result_filter_reinforce",filterImage);
//        imshow("Blue",bblueImage);
//        imshow("Red",bredImage);
//        imshow("Green",bgreenImage);
        waitKey(1);
	}

	void threshdouble(Mat &inputImage,Mat &outputImage,int thresh_low,int thresh_high)
    {
        Mat sumImage,highImage,lowImage;

	    threshold(inputImage, highImage,thresh_high, 255, CV_THRESH_BINARY);
        threshold(inputImage, lowImage,thresh_low, 255, CV_THRESH_BINARY);
        integral(highImage,sumImage);
//        imshow("highthresh", highImage);
//        imshow("lowthresh", lowImage);
//        imshow("sumImage", sumImage);
        int nr=lowImage.rows;
        int nc=lowImage.cols;
        //TODO 积分图计算patch的大小;影响边缘无点范围
        int ssize=20;
		for(int i=0;i<nr;i++)
		{
//			auto lowData=lowImage.ptr<uchar>(i);  //const uchar*
//            auto highData1=sumImage.ptr<uchar>(i-ssize);
//            auto highData2=sumImage.ptr<uchar>(i+ssize);
			for(int j=0;j<nc;j++) {
				if(lowImage.at<uchar>(i,j)>0)
                {
                    if(i<ssize||j<ssize||i>=nr-ssize||j>=nc-ssize)
                    {
                        lowImage.at<uchar>(i,j)=0;
                    } else{
                        int sunnum=sumImage.at<int>(i+ssize,j+ssize)-sumImage.at<int>(i-ssize,j+ssize)-sumImage.at<int>(i+ssize,j-ssize)+sumImage.at<int>(i-ssize,j-ssize);
                        //TODO 参数影响大
                        if(sunnum<20*255)
                        {
                            lowImage.at<uchar>(i,j)=0;
                        }
                    }


                }

                //if(1)//highData2[j+ssize]-highData1[j-ssize]<1*255)  //TODO 附近没有高阈值点
//                    {
//                        lowData[j]=255;
//                    }
			}
		}
        outputImage=lowImage.clone();

        //膨胀操作
//        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//        dilate(rHH, rHH, element);
    }

    void depthouter(Mat &image)
    {
        int nr=depthImage.rows;
        int nc=depthImage.cols;
        for(int i=0;i<nr;i++) {
            for (int j = 0; j < nc; j++) {
                if (image.at<uchar>(i, j) > 0) {
                    if(depthImage.at<ushort>(i, j)>7000){//mm为单位  5×8
                        image.at<uchar>(i, j)=0;
                    }
                    if(eleImage.at<uchar>(i,j)>0){
                        image.at<uchar>(i, j)=0;
                    }
                }
            }
        }
    }


//    void transto3D(float &point[3], const float pixel[2], float depth)
//    {
//        assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
//        assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
//        //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model
//
//        float x = (pixel[0] - intrin->ppx) / intrin->fx;
//        float y = (pixel[1] - intrin->ppy) / intrin->fy;
//        if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
//        {
//            float r2  = x*x + y*y;
//            float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
//            float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
//            float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
//            x = ux;
//            y = uy;
//        }
//        point[0] = depth * x;
//        point[1] = depth * y;
//        point[2] = depth;
//    rs2_deproject_pixel_to_point();}


    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        depthImage=cv_ptr->image;
        eleImage=Mat::zeros(480,640,CV_8UC1);
        for (int m = 0; m < depthImage.rows; m++){
            for (int n = 0; n < depthImage.cols; n++){
                // 获取深度图中(m,n)处的值
                double d = depthImage.ptr<ushort>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
                // d 可能没有值，若如此，跳过此点
                if (d == 0)
                    continue;

                // 计算这个点的空间坐标
                //float z = d/ camera_factor;
                //disImage.ptr<float>(m)[n]= (n - camera_cx) * z / camera_fx;
                auto Yw = ((camera_cy - m)* d/ camera_fy+250);
                if(Yw<0||d==0)
                    eleImage.ptr<uchar>(m)[n]=255;

                //(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
//                p.b = color_pic.ptr<uchar>(m)[n*3];
//                p.g = color_pic.ptr<uchar>(m)[n*3+1];
//                p.r = color_pic.ptr<uchar>(m)[n*3+2];

            }
        }
        imshow("depthImage",depthImage*15);  //显示效果乘15
        imshow("eleImage",eleImage);
//        imshow("disImage",disImage);
        waitKey(1);
    }
	void colorCallback(const sensor_msgs::ImageConstPtr& msg)
	{
	    time=(double)getTickCount();
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		Mat sgrayImage;
		srcImage=cv_ptr->image;
		label_image=Mat::zeros(srcImage.rows/grid[0],srcImage.cols/grid[1], CV_8UC1);
		Search_state=Mat::zeros(label_image.rows,label_image.cols, CV_8UC1);
		cvtColor(srcImage,sgrayImage,CV_BGR2GRAY);


//		float IH[] = { -0.125,0.0,0.125,-0.25,0.0,0.25,-0.125,0.0,0.125 };
//		float HI[] = { -0.125,-0.25,-0.125,0.0,0.0,0.0,0.125,0.25,0.125 };
		float HH[] = { 0.25,0.0,-0.25,0.0,0.0,0.0,-0.25,0.0,0.25 };
		Mat matHI, matHH, matIH;
		Mat rHI, rHH, rIH;

//		Mat KerHI( 3, 3, CV_32FC1, HI);
//		Mat KerIH(3, 3, CV_32FC1, IH);
		Mat KerHH( 3, 3, CV_32FC1, HH);

		//十分耗时
//		filter2D(sgrayImage,matHI, CV_32FC1,KerHI);
//		filter2D(sgrayImage, matIH, CV_32FC1, KerIH);
		filter2D(sgrayImage, matHH, CV_8UC1, KerHH);

//		threshold(matHI, rHI, 7, 255, CV_THRESH_BINARY);
//		threshold(matIH, rIH, 7, 255, CV_THRESH_BINARY);
//		threshold(matHH, rHH, 8, 255, CV_THRESH_BINARY);
        threshdouble(matHH, rHH, 6, 9);
//        filter_reinforce(matHH);
//		threshdouble(matHH, rHH, 4, 90);

//        adaptiveThreshold(matHH, rHH, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 25, 25);
        depthouter(rHH);
        result_filter=rHH.clone();

//		imshow("src", srcImage);
		imshow("reinforce", result_filter);


        //waitKey(1);


        BFS_seg();


        //_shiTomasi();

	}

protected:
	ros::NodeHandle nh;
	//ros::Publisher pub1,pub2;
	ros::Subscriber sub;
    ros::Subscriber sub_depth;
	std::vector<class_label> labelpoints;
    std::vector<class_label> mergepoints;
	Mat label_image;
	Mat Search_state;
	Mat srcImage=Mat::zeros(480,640,CV_8UC3);;
	Mat depthImage=Mat::zeros(480,640,CV_16UC1);
	Mat eleImage=Mat::zeros(480,640,CV_8UC1);
	Mat disImage=Mat::zeros(480,640,CV_16UC1);
	int grid[2]={12,16};  //24.32   块的大小影响和周边点的区分
	Mat result_filter;
	double time;
	std::vector<class_label> lastbox;
    const float camera_factor = 1;
    const float camera_cx = 328.284f;
    const float camera_cy = 240.539f;
    const float camera_fx = 612.924f;
    const float camera_fy = 612.74f;
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
