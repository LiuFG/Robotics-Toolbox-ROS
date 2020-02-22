//
// Created by lfg on 19-4-1.
//

#ifndef FILTER_CATCH_KEY_POINTS_H
#define FILTER_CATCH_KEY_POINTS_H
//#include <opencv2/xfeatures2d.hpp>
#include "harris.h"
using namespace cv;
void _harris(Mat srcImage)
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
void _shiTomasi(Mat srcImage)
{
    Mat cornerTomasi_result,grayImage;
    cvtColor(srcImage,grayImage,CV_BGR2GRAY);
    cornerTomasi_result=grayImage.clone();
    //=============shi-Tomasi=============
    std::vector<Point> cornersTomasi;
    goodFeaturesToTrack(cornerTomasi_result,cornersTomasi,200,0.01,10);
    harris().drawOnImage(srcImage,cornersTomasi);

//    //FPS
//    timemerge=((double)getTickCount()-timemerge)/getTickFrequency();
//    double fps =1.0/timemerge;
//    char num_fps[10];
//    sprintf(num_fps,"%.2f",fps);
//    std::string sfps("FPS:");
//    sfps+=num_fps;
//    putText(srcImage,sfps,Point(5,20),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255));
    cv::namedWindow ("shi-Tomasi");
    cv::imshow ("shi-Tomasi",srcImage);
    waitKey(1);

}
//    void _FAST(Mat srcImage)
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
void _SIFT(Mat srcImage)
{   //TODO sift检测彩色图
    Mat grayImage,sift_srcImage;
    //cvtColor(srcImage,grayImage,CV_BGR2GRAY);
    sift_srcImage=srcImage.clone();
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
//    void _SURF(Mat srcImage)
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


#endif //FILTER_CATCH_KEY_POINTS_H
