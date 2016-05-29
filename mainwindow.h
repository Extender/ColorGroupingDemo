#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <ctime>
#include <map>

#include <QMainWindow>
#include <QFile>
#include <QImage>
#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>

#include "mainwindowex.h"
#include "graphicsviewex.h"
#include "graphicssceneex.h"

namespace Ui {
class MainWindow;
}

struct Point
{
    int32_t x;
    int32_t y;

    Point();
    Point(int32_t _x,int32_t _y);
};

class MainWindow : public MainWindowEx
{
    Q_OBJECT

    QFileDialog *dialog;
    QString currentFile;
    QImage *originalImage;
    QImage *filteredImage;
    GraphicsSceneEx *scene;
    QGraphicsPixmapItem *pixmapItem;
    uint32_t *bmpData;
    std::map<uint32_t,uint32_t> *areaColors;
    uint32_t *pixelAreas;
    uint32_t *areaRedirects;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QImage *getImageFromBmpData(int32_t width,int32_t height,uint32_t *data);
    static double *applyGaussianBlurToSingleChannelDoubleArray(double *in,int32_t width,int32_t height,int32_t filterSize,double deviation);
    static uint32_t *getImageFromBWDoubleArray(double *in,int32_t width,int32_t height);
    static uint32_t *qImageToBitmapData(QImage *image);
    static uint32_t *getImageFromRGBChannelDoubleArrays(double *rChannel,double *gChannel,double *bChannel,int32_t width,int32_t height);
    static uint32_t *getImageFromARGBChannelDoubleArrays(double *aChannel,double *rChannel,double *gChannel,double *bChannel,int32_t width,int32_t height);
    static double *getBWDoubleArrayFromImage(uint32_t *image,int32_t width,int32_t height);
    static double *getAlphaChannelDoubleArrayFromImage(uint32_t *image,int32_t width,int32_t height);
    static double *getRedChannelDoubleArrayFromImage(uint32_t *image,int32_t width,int32_t height);
    static double *getGreenChannelDoubleArrayFromImage(uint32_t *image,int32_t width,int32_t height);
    static double *getBlueChannelDoubleArrayFromImage(uint32_t *image,int32_t width,int32_t height);
    static void getRGBChannelsFromImage(uint32_t *image,int32_t width,int32_t height,double *&rChannel,double *&gChannel,double *&bChannel);
    static void getARGBChannelsFromImage(uint32_t *image,int32_t width,int32_t height,double *&aChannel,double *&rChannel,double *&gChannel,double *&bChannel);
    static uint32_t getRandomColor();
    static int32_t round(double in);

public slots:
    void browseButtonClicked(bool checked);
    void fileSelected(QString file);
    void groupAreasBtnClicked(bool clicked);
    void resetBtnClicked(bool checked);
    void fitToWindow();
    void resetZoom();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
