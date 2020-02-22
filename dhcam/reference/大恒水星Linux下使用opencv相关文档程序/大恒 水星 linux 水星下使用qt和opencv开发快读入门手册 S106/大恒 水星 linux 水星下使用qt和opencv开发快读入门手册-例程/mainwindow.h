#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "GxIAPI.h"
#include "DxImageProc.h"

#include <opencv2/core.hpp>

#include <opencv2/highgui.hpp>

using namespace cv;


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public:
    GX_STATUS status;
    GX_DEV_HANDLE m_hDevice;
    bool m_Is_implemented;
    int64_t m_pixel_color;
    char *m_rgb_image;
    Mat m_image;
    void ShowErrorString(GX_STATUS emErrorStatus);
    static  void  OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);



signals:
void drawImageSignal();

private slots:
    void on_OpenButton_clicked();
    void on_CLoseButton_clicked();
    void drawImageSlots();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
