//-------------------------------------------------------------
/**
\file      GxAcquireContinuous.cpp
\brief     sample to show how to acquire image continuously. 
\version   1.0.1605.9041
\date      2016-05-04
*/
//-------------------------------------------------------------

#include "GxIAPI.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "DxImageProc.h"
using namespace std;
using namespace cv;
Mat m_image;
bool      is_implemented = false;
int64_t                    m_pixel_color = 0;              ///< Bayer格式
char*m_rgb_image = NULL;

#define MEMORY_ALLOT_ERROR -1 

GX_DEV_HANDLE g_device = NULL;              ///< 设备句柄
GX_FRAME_DATA g_frame_data = { 0 };         ///< 采集图像参数
pthread_t g_acquire_thread = 0;             ///< 采集线程ID
bool g_get_image = false;                   ///< 采集线程是否结束的标志：true 运行；false 退出

//获取图像大小并申请图像数据空间
int PreForImage();

//释放资源
int UnPreForImage();

//采集线程函数
void *ProcGetImage(void* param);

//获取错误信息描述
void GetErrorString(GX_STATUS error_status);

int main()
{
    uid_t user = 0;
    user = geteuid();
    if(user != 0)
    {
        printf("\n");  
        printf("Please run this application with 'sudo -E ./GxAcquireContinuous' or"
                              " Start with root !\n");
        printf("\n");
        return 0;
    }

    printf("\n");
    printf("-------------------------------------------------------------\n");
    printf("sample to show how to acquire image continuously.\n");
    #ifdef __x86_64__
    printf("version: 1.0.1605.8041\n");
    #elif __i386__
    printf("version: 1.0.1605.9041\n");
    #endif
    printf("-------------------------------------------------------------\n");
    printf("\n");

    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("Initializing......"); 
    printf("\n\n");
    
    
    usleep(2000000);   
     
    //API接口函数返回值 
    GX_STATUS status = GX_STATUS_SUCCESS;

    uint32_t device_num = 0;
    uint32_t ret = 0;
    GX_OPEN_PARAM open_param;

    //初始化设备打开参数，默认打开序号为1的设备
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = "1";

    //初始化库
    status = GXInitLib(); 
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return 0;
    }

    //获取枚举设备个数
    status = GXUpdateDeviceList(&device_num, 1000);
    if(status != GX_STATUS_SUCCESS)
    { 
        GetErrorString(status);
        status = GXCloseLib();
        return 0;
    }

    if(device_num <= 0)
    {
        printf("<No device>\n");
        status = GXCloseLib();
        return 0;
    }
    else
    {
        //默认打开第1个设备
        status = GXOpenDevice(&open_param, &g_device);
        if(status == GX_STATUS_SUCCESS)
	{
            printf("<Open device success>\n");
	    int64_t width,height;
    	    status = GXGetInt(g_device,GX_INT_WIDTH,&width);
    	    status = GXGetInt(g_device,GX_INT_HEIGHT,&height);
	    // 查询当前相机是否支持GX_ENUM_PIXEL_COLOR_FILTER
    	    status = GXIsImplemented(g_device, GX_ENUM_PIXEL_COLOR_FILTER, &is_implemented);

   	   //支持彩色图像
    	   if(is_implemented)
    	   {
       		status = GXGetEnum(g_device, GX_ENUM_PIXEL_COLOR_FILTER, &m_pixel_color);
		m_image.create(height,width,CV_8UC3);
		m_rgb_image = new char[width*height*3];
   	   }else{
	    
 	    	m_image.create(height,width,CV_8UC1);
	   }
        }
        else
        {
            printf("<Open device fail>\n");
            status = GXCloseLib();
            return 0;			
        }
    }

    //设置采集模式为连续采集
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //设置触发开关为OFF
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
	
    //为采集做准备    
    ret = PreForImage();
    if(ret != 0)    
    {
        printf("<Failed to prepare for acquire image>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //启动接收线程
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    if(ret != 0)
    {
        printf("<Failed to create the collection thread>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    bool run = true;
    while(run == true)
    {
        int c = getchar();

        switch(c)
        {
            //退出程序
            case 'X': 
            case 'x':
                run = false;
                break;
            default:
                break;
        }	
    }

    //为停止采集做准备
    ret = UnPreForImage();
    if(ret != 0)
    {
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //关闭设备
    status = GXCloseDevice(g_device);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //释放库
    status = GXCloseLib();
    return 0;
}

//-------------------------------------------------
/**
\brief 获取图像大小并申请图像数据空间
\return void
*/
//-------------------------------------------------
int PreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    int64_t payload_size = 0;
	
    status = GXGetInt(g_device, GX_INT_PAYLOAD_SIZE, &payload_size);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }
	
    g_frame_data.pImgBuf = malloc(payload_size);
    if(g_frame_data.pImgBuf == NULL)
    {
        printf("<Failed to allot memory>\n");
        return MEMORY_ALLOT_ERROR;
    }
 
    return 0;
}

//-------------------------------------------------
/**
\brief 释放资源
\return void
*/
//-------------------------------------------------
int UnPreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ret = 0;
   
    //发送停采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_STOP);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }

    g_get_image = false;
    ret = pthread_join(g_acquire_thread,NULL);
    if(ret != 0)
    {
        printf("<Failed to release resources>\n");
        return ret;
    }
	

    //释放buffer
    if(g_frame_data.pImgBuf != NULL)
    {
        free(g_frame_data.pImgBuf);
        g_frame_data.pImgBuf = NULL;
    }

    return 0;
}

//-------------------------------------------------
/**
\brief 采集线程函数
\param pParam 线程传入参数
\return void*
*/
//-------------------------------------------------
void *ProcGetImage(void* pParam)
{
    GX_STATUS status = GX_STATUS_SUCCESS;

    //接收线程启动标志
    g_get_image = true;

    //发送开采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_START);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }
	
    while(g_get_image)
    {
        if(g_frame_data.pImgBuf == NULL)
        {
            continue;
        }

        status = GXGetImage(g_device, &g_frame_data, 100);
        if(status == GX_STATUS_SUCCESS)
        {
            if(g_frame_data.nStatus == 0)
            {
                printf("<Successful acquisition : Width: %d Height: %d >\n", g_frame_data.nWidth, g_frame_data.nHeight);
		if(is_implemented)
    	   	{
/*
2016DaHeng
/// Bayer layout
typedef enum  tagDX_PIXEL_COLOR_FILTER
{
	NONE    = 0,   
	BAYERRG = 1,   
	BAYERGB = 2,   
	BAYERGR = 3,   
	BAYERBG = 4    
} DX_PIXEL_COLOR_FILTER;

*/
		DxRaw8toRGB24(g_frame_data.pImgBuf, m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
       			memcpy(m_image.data,m_rgb_image,g_frame_data.nHeight*g_frame_data.nWidth*3);
   	   	}else{
	    
 	    		memcpy(m_image.data,g_frame_data.pImgBuf,g_frame_data.nHeight*g_frame_data.nWidth);
	   	}
            	namedWindow("test");
           	imshow("test", m_image);
            	waitKey(5);
            }
        }
    }
}

//----------------------------------------------------------------------------------
/**
\brief  获取错误信息描述
\param  emErrorStatus  错误码

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS error_status)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS status = GX_STATUS_SUCCESS;
	
    // 获取错误描述信息长度
    status = GXGetLastError(&error_status, NULL, &size);
    if(status != GX_STATUS_SUCCESS)
    {
           GetErrorString(status);
	   return;
    }
	
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
	
    // 获取错误信息描述
    status = GXGetLastError(&error_status, error_info, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        printf("<GXGetLastError call fail>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // 释放资源
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

