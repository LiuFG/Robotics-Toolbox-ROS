//
// Created by lfg on 19-3-29.
//

#ifndef FILTER_CATCH_CLASSIFY_H
#define FILTER_CATCH_CLASSIFY_H

#include <iostream>
#include "vector"

struct point2D{
   int x=0;
   int y=0;
};

struct class_label
{
    class_label()
    {
        vertex_min.x=2000;
        vertex_min.y=2000;
        vertex_max.x=0;
        vertex_max.y=0;
        label=0;
        centerpoint.x=0;
        centerpoint.y=0;
        width=0;
        height=0;
        prob=0;
        point_num=0;
        depth_value=10000;
    }
    //std::vector<point2D> points;
    point2D vertex_min;
    point2D vertex_max;
    int label;
    point2D centerpoint;
    int width;
    int height;
    float prob;
    int point_num;
    ushort depth_value; //mm  除0的最近点来描述
};


#endif //FILTER_CATCH_CLASSIFY_H
