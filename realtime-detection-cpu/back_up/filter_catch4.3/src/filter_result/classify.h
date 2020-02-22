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
    }
    std::vector<point2D> points;
    point2D vertex_min;
    point2D vertex_max;
    int label;
};




#endif //FILTER_CATCH_CLASSIFY_H
