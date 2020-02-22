//
// Created by lfg on 19-4-13.
//

#ifndef FILTER_CATCH_IFOCCUPY_H
#define FILTER_CATCH_IFOCCUPY_H

bool if_occupy( int grid_row,int grid_col)
{
    int flag=0;
    int omergeflag=0;
    int flagnum_lu=0;
    int flagnum_ld=0;
    int flagnum_ru=0;
    int flagnum_rd=0;
    int judgenum=1;//max(int(floor(float(grid[0])/10)),1);
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

    if(omergeflag>3)//(flagnum_rd>judgenum&&flagnum_ru>judgenum)||(flagnum_ld>judgenum&&flagnum_lu>judgenum))
    {
        //ROS_INFO("it is occupied");
        return true;
    }
    else
        return false;
}
#endif //FILTER_CATCH_IFOCCUPY_H
