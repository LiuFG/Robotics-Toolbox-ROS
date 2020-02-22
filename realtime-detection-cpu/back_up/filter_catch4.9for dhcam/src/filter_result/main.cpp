/*=================================================
 * Editor:
===================================================
*/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "classify.h"
#include <iostream>
#include <queue>
#include <stdlib.h>
#include "harris.h"
#include "key_points.h"

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
						labelpoints[vec_num].vertex_min.x=min(memp.x,labelpoints[vec_num].vertex_min.x);
						labelpoints[vec_num].vertex_min.y=min(memp.y,labelpoints[vec_num].vertex_min.y);
						labelpoints[vec_num].vertex_max.x=max(memp.x,labelpoints[vec_num].vertex_max.x);
						labelpoints[vec_num].vertex_max.y=max(memp.y,labelpoints[vec_num].vertex_max.y);
						//TODO 加速：不压点
						//labelpoints[vec_num].points.push_back(memp);
						labelpoints[vec_num].point_num++;
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
						grid_points.vertex_min.x=min(memp.x,grid_points.vertex_min.x);
						grid_points.vertex_min.y=min(memp.y,grid_points.vertex_min.y);
						grid_points.vertex_max.x=max(memp.x,grid_points.vertex_max.x);
						grid_points.vertex_max.y=max(memp.y,grid_points.vertex_max.y);
						//TODO 加速：不压点
						//grid_points.points.push_back(memp);
						grid_points.point_num++;
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
		int judgenum=5;
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
		//右上
		for(int k=0;k<grid[0]/2;k++)
		{
			for(int i=grid[1]/2;i<grid[1];i++)
			{
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
				flag=result_filter.at<uchar>(grid_row*grid[0]+k,grid_col*grid[1]+i);
				//std::cout<<"filter ingrid value:"<<flag<<std::endl;
				if(flag>0)
					flagnum_rd++;
				if(flagnum_rd>judgenum)
				{
					omergeflag++;
					break;
				}
			}
		}

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
			//TODO 参数
			if(iter->height*iter->width>1600)
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
			miter->prob=1-abs(float(miter->width/miter->height)-1);
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
                    {
                        float dis_x=abs(iter->centerpoint.x-iiter->centerpoint.x);
                        float dis_y=abs(iter->centerpoint.y-iiter->centerpoint.y);

                        //TODO 参数影响大
                        if(dis_x*2/(iter->height+iiter->height)<2.5&&dis_y*2/(iter->width+iiter->width)<2.5)
                        {
                            //更新labelpoints
                            iter->vertex_min.x=min(iter->vertex_min.x,iiter->vertex_min.x);
                            iiter->vertex_min.x=iter->vertex_min.x;
                            iter->vertex_min.y=min(iter->vertex_min.y,iiter->vertex_min.y);
                            iiter->vertex_min.y=iter->vertex_min.y;

                            iter->vertex_max.x=max(iter->vertex_max.x,iiter->vertex_max.x);
                            iiter->vertex_max.x=iter->vertex_max.x;
                            iter->vertex_max.y=max(iter->vertex_max.y,iiter->vertex_max.y);
                            iiter->vertex_max.y=iter->vertex_max.y;

                            //更新其他参数
                            iter->centerpoint.x=(iter->vertex_min.x+iter->vertex_max.x)/2;
                            iter->centerpoint.y=(iter->vertex_min.y+iter->vertex_max.y)/2;
                            iter->width=iter->vertex_max.y-iter->vertex_min.y;
                            iter->height=iter->vertex_max.x-iter->vertex_min.x;
                            iiter->centerpoint.x=(iiter->vertex_min.x+iiter->vertex_max.x)/2;
                            iiter->centerpoint.y=(iiter->vertex_min.y+iiter->vertex_max.y)/2;
                            iiter->width=iiter->vertex_max.y-iiter->vertex_min.y;
                            iiter->height=iiter->vertex_max.x-iiter->vertex_min.x;

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
        //TODO 参数影响大,太小就把车上的特征点也去掉了
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

	void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
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

        result_filter=rHH.clone();

//		imshow("HI", rHH);
//		imshow("reinforce", result_filter);


        //waitKey(1);


        BFS_seg();


        //_shiTomasi();

	}

protected:
	ros::NodeHandle nh;
	//ros::Publisher pub1,pub2;
	ros::Subscriber sub;
	std::vector<class_label> labelpoints;
    std::vector<class_label> mergepoints;
	Mat label_image;
	Mat Search_state;
	Mat srcImage;
	int grid[2]={32,32};
	Mat result_filter;
	double time;
	std::vector<class_label> lastbox;
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