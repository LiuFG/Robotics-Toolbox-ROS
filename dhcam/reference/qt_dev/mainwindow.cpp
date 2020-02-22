#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this,SIGNAL(drawImageSignal()),this,SLOT(drawImageSlots()));
    status = GXInitLib();
}

MainWindow::~MainWindow()
{
    status = GXCloseLib();
    delete ui;
}

void MainWindow::drawImageSlots()

{
    imshow("test", m_image);
}


void  MainWindow::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if(pFrame->status)
    return;

    MainWindow * main = (MainWindow *) pFrame->pUserParam;
    if(main->m_Is_implemented)
    {
        DxRaw8toRGB24((void*)pFrame->pImgBuf,main->m_rgb_image,pFrame->nWidth, pFrame->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERGB),false); memcpy(main->m_image.data,main->m_rgb_image,pFrame->nHeight*pFrame->nWidth*3);
    }
    else
    {
        memcpy(main->m_image.data,pFrame->pImgBuf,pFrame->nHeight*pFrame->nWidth);
    }

    //send draw message

    main->drawImageSignal();//自定义消息看第八步
}

void MainWindow::on_OpenButton_clicked()
{
    //枚举设备个数

    uint32_t nDeviceNum = 0;
    status =GXUpdateDeviceList(&nDeviceNum, 1000);
    if(nDeviceNum <= 0)
    return;
    GX_OPEN_PARAM openParam;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;//访问方式
    openParam.openMode = GX_OPEN_INDEX; //通过序列号
    openParam.pszContent = "1";
    //open camera

    status = GXOpenDevice(&openParam, &m_hDevice);
    //ShowErrorString(status);
    if(status)
    return;

    ////////////////////////init opencv////////////////////

    int64_t width,height;
    status = GXGetInt(m_hDevice,GX_INT_WIDTH,&width);
    status = GXGetInt(m_hDevice,GX_INT_HEIGHT,&height);

    // 查询当前相机是否支持GX_ENUM_PIXEL_COLOR_FILTER
    status=GXIsImplemented(m_hDevice,GX_ENUM_PIXEL_COLOR_FILTER, &m_Is_implemented);
    //支持彩色图像
    if(m_Is_implemented)
    {
        status= GXGetEnum(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_pixel_color);
        m_image.create(height,width,CV_8UC3);
        m_rgb_image = new char[width*height*3];
    }
    else
    {
        m_image.create(height,width,CV_8UC1);
    }

    ////////////////////////init opencv////////////////////

    //注册图像处理回调函数
    status = GXRegisterCaptureCallback(m_hDevice, this, (GXCaptureCallBack)OnFrameCallbackFun);

    //发送开采命令
    status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);

}

void MainWindow::on_CLoseButton_clicked()
{
    //发送停采命令
    status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);

    //注销采集回调
    status = GXUnregisterCaptureCallback(m_hDevice);
    status = GXCloseDevice(m_hDevice); //close device

    if(m_rgb_image)
    {
        delete m_rgb_image;
        m_rgb_image = NULL;
    }
}
