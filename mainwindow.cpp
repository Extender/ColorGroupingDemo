#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    MainWindowEx(parent),
    ui(new Ui::MainWindow)
{
    srand(time(0));

    ui->setupUi(this);
    scene=new GraphicsSceneEx();
    pixmapItem=new QGraphicsPixmapItem();
    scene->addItem(pixmapItem);
    ui->graphicsView->setScene(scene);
    connect(this,SIGNAL(windowResizedEx(QResizeEvent*)),this,SLOT(fitToWindow()));
    connect(ui->browseButton,SIGNAL(clicked(bool)),this,SLOT(browseButtonClicked()));
    connect(ui->groupPixelsBtn,SIGNAL(clicked(bool)),this,SLOT(groupAreasBtnClicked()));
    connect(ui->resetBtn,SIGNAL(clicked(bool)),this,SLOT(resetBtnClicked()));
    dialog=new QFileDialog(this);
    dialog->setNameFilter("All images (*.jpg *.jpeg *.png *.gif *.bmp *.dib *.tif *.tiff)");
    dialog->setDirectory(QApplication::applicationDirPath());
    connect(dialog,SIGNAL(fileSelected(QString)),this,SLOT(fileSelected(QString)));

    originalImage=0;
    filteredImage=0;
    bmpData=0;

    ui->colorGroupingKernelSizeBox->setValue(1);
    ui->colorGroupingToleranceBox->setValue(0.2);

    QString proposedFile=QApplication::applicationDirPath()+"/example.jpg";
    QFile f(proposedFile);
    if(f.exists())
    {
        currentFile=proposedFile;
        ui->lineEdit->setText(currentFile);
        fileSelected(currentFile);
    }
    else
        currentFile="";

    areaRedirects=0;
    pixelAreas=0;
    areaColors=0;
}

MainWindow::~MainWindow()
{
    delete ui;
    delete originalImage;
    delete filteredImage;
    delete areaColors;
    free(areaRedirects);
    free(pixelAreas);
    free(bmpData);
}

QImage *MainWindow::getImageFromBmpData(int32_t width, int32_t height, uint32_t *data)
{
    QImage *image=new QImage(width,height,QImage::Format_ARGB32);
    image->fill(0xFFFFFFFF);
    for(int32_t y=0;y<height;y++)
    {
        for(int32_t x=0;x<width;x++)
        {
            image->setPixel(x,y,data[y*width+x]);
        }
    }
    return image;
}

double *MainWindow::applyGaussianBlurToSingleChannelDoubleArray(double *in, int32_t width, int32_t height, int32_t filterSize, double deviation)
{
    int32_t filterSizeInPixels=2*filterSize+1;
    int32_t filterAreaInPixels=filterSizeInPixels*filterSizeInPixels;
    double *out=(double*)malloc(width*height*sizeof(double));

    double *filterFactors=(double*)malloc(filterAreaInPixels*sizeof(double));

    for(int32_t filterY=0;filterY<filterSizeInPixels;filterY++)
    {
        for(int32_t filterX=0;filterX<filterSizeInPixels;filterX++)
        {
            double factor=(1.0/(2.0*M_PI*pow(deviation,2.0)))*exp(-1.0*((pow(((double)(filterX+1))-(((double)filterSize)+1.0),2.0)+pow(((double)(filterY+1))-(((double)filterSize)+1.0),2.0))/(2.0*pow(deviation,2.0))));
            filterFactors[filterY*filterSizeInPixels+filterX]=factor;
        }
    }

    for(int32_t y=0;y<height;y++)
    {
        for(int32_t x=0;x<width;x++)
        {
            double pixelValueSum=0.0;

            // Convolve using this pixel as the center

            for(int32_t yOfFilter=0;yOfFilter<filterSizeInPixels;yOfFilter++)
            {
                int32_t yWithFilter=-filterSize+y+yOfFilter;
                 // Use symmetry to compensate for missing pixels (in order to avoid dark borders)
                if(yWithFilter<0)
                    yWithFilter=(y+filterSize)-yOfFilter;
                else if(yWithFilter>=height)
                    yWithFilter=(y-filterSize)+(filterSizeInPixels-1-yOfFilter);
                for(int32_t xOfFilter=0;xOfFilter<filterSizeInPixels;xOfFilter++)
                {
                    int32_t xWithFilter=-filterSize+x+xOfFilter;
                    // Use symmetry to compensate for missing pixels (in order to avoid dark borders)
                    if(xWithFilter<0)
                        xWithFilter=(x+filterSize)-xOfFilter;
                    else if(xWithFilter>=width)
                        xWithFilter=(x-filterSize)+(filterSizeInPixels-1-xOfFilter);

                    double factor=filterFactors[yOfFilter*filterSizeInPixels+xOfFilter];

                    pixelValueSum+=in[yWithFilter*width+xWithFilter]*factor;
                }
            }

            out[y*width+x]=pixelValueSum;
        }
    }
    return out;
}

uint32_t *MainWindow::getImageFromBWDoubleArray(double *in, int32_t width, int32_t height)
{
    uint32_t *out=(uint32_t*)malloc(width*height*sizeof(uint32_t));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            int32_t pos=offset+x;
            double pixelValue=in[pos];
            uint32_t component=(uint32_t)round(pixelValue*255);
            uint32_t outColor=0xFF000000;
            outColor|=(component<<16)|(component<<8)|component;
            out[pos]=outColor;
        }
    }
    return out;
}

uint32_t *MainWindow::qImageToBitmapData(QImage *image)
{
    int32_t width=image->width();
    int32_t height=image->height();
    uint32_t *out=(uint32_t*)malloc(width*height*sizeof(uint32_t));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        QRgb *scanLine=(QRgb*)image->scanLine(y); // Do not free!
        for(int32_t x=0;x<width;x++)
        {
            QRgb color=scanLine[x];
            uint32_t alpha=qAlpha(color);
            uint32_t red=qRed(color);
            uint32_t green=qGreen(color);
            uint32_t blue=qBlue(color);
            out[offset+x]=(alpha<<24)|(red<<16)|(green<<8)|blue;
        }
        // Do not free "scanLine"!
    }
    return out;
}

uint32_t *MainWindow::getImageFromARGBChannelDoubleArrays(double *aChannel, double *rChannel, double *gChannel, double *bChannel, int32_t width, int32_t height)
{
    uint32_t *out=(uint32_t*)malloc(width*height*sizeof(uint32_t));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            int32_t pos=offset+x;
            uint32_t alpha=round(aChannel[pos]*255.0);
            uint32_t red=round(rChannel[pos]*255.0);
            uint32_t green=round(gChannel[pos]*255.0);
            uint32_t blue=round(bChannel[pos]*255.0);
            uint32_t outColor=(alpha<<24)|(red<<16)|(green<<8)|blue;
            out[pos]=outColor;
        }
    }
    return out;
}

uint32_t *MainWindow::getImageFromRGBChannelDoubleArrays(double *rChannel, double *gChannel, double *bChannel, int32_t width, int32_t height)
{
    uint32_t *out=(uint32_t*)malloc(width*height*sizeof(uint32_t));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            int32_t pos=offset+x;
            uint32_t red=round(rChannel[pos]*255.0);
            uint32_t green=round(gChannel[pos]*255.0);
            uint32_t blue=round(bChannel[pos]*255.0);
            uint32_t outColor=0xFF000000;
            outColor|=(red<<16)|(green<<8)|blue;
            out[pos]=outColor;
        }
    }
    return out;
}

double *MainWindow::getBWDoubleArrayFromImage(uint32_t *image, int32_t width, int32_t height)
{
    double *out=(double*)malloc(width*height*sizeof(double));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t color=image[offset+x];
            double r=((double)((color>>16)&0xff))/255.0;
            double g=((double)((color>>8)&0xff))/255.0;
            double b=((double)(color&0xff))/255.0;
            out[offset+x]=(0.2126*r+0.7152*g+0.0722*b);
        }
    }
    return out;
}

double *MainWindow::getAlphaChannelDoubleArrayFromImage(uint32_t *image, int32_t width, int32_t height)
{
    double *out=(double*)malloc(width*height*sizeof(double));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t color=image[offset+x];
            double value=((double)((color>>24)&0xff))/255.0;
            out[offset+x]=value;
        }
    }
    return out;
}

double *MainWindow::getRedChannelDoubleArrayFromImage(uint32_t *image, int32_t width, int32_t height)
{
    double *out=(double*)malloc(width*height*sizeof(double));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t color=image[offset+x];
            double value=((double)((color>>16)&0xff))/255.0;
            out[offset+x]=value;
        }
    }
    return out;
}

double *MainWindow::getGreenChannelDoubleArrayFromImage(uint32_t *image, int32_t width, int32_t height)
{
    double *out=(double*)malloc(width*height*sizeof(double));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t color=image[offset+x];
            double value=((double)((color>>8)&0xff))/255.0;
            out[offset+x]=value;
        }
    }
    return out;
}

double *MainWindow::getBlueChannelDoubleArrayFromImage(uint32_t *image, int32_t width, int32_t height)
{
    double *out=(double*)malloc(width*height*sizeof(double));
    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t color=image[offset+x];
            double value=((double)(color&0xff))/255.0;
            out[offset+x]=value;
        }
    }
    return out;
}

void MainWindow::getRGBChannelsFromImage(uint32_t *image, int32_t width, int32_t height, double *&rChannel, double *&gChannel, double *&bChannel)
{
    uint32_t widthAndHeightBasedDoubleArraySize=width*height*sizeof(double);
    rChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);
    gChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);
    bChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);

    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t pos=offset+x;
            uint32_t color=image[pos];
            double r=((double)((color>>16)&0xff))/255.0;
            double g=((double)((color>>8)&0xff))/255.0;
            double b=((double)(color&0xff))/255.0;
            rChannel[pos]=r;
            gChannel[pos]=g;
            bChannel[pos]=b;
        }
    }
}

void MainWindow::getARGBChannelsFromImage(uint32_t *image, int32_t width, int32_t height, double *&aChannel, double *&rChannel, double *&gChannel, double *&bChannel)
{
    uint32_t widthAndHeightBasedDoubleArraySize=width*height*sizeof(double);
    aChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);
    rChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);
    gChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);
    bChannel=(double*)malloc(widthAndHeightBasedDoubleArraySize);

    for(int32_t y=0;y<height;y++)
    {
        int32_t offset=y*width;
        for(int32_t x=0;x<width;x++)
        {
            uint32_t pos=offset+x;
            uint32_t color=image[pos];
            double a=((double)((color>>24)&0xff))/255.0;
            double r=((double)((color>>16)&0xff))/255.0;
            double g=((double)((color>>8)&0xff))/255.0;
            double b=((double)(color&0xff))/255.0;
            aChannel[pos]=a;
            rChannel[pos]=r;
            gChannel[pos]=g;
            bChannel[pos]=b;
        }
    }
}

uint32_t MainWindow::getRandomColor()
{
    double r=((double)rand())/((double)RAND_MAX);
    double g=((double)rand())/((double)RAND_MAX);
    double b=((double)rand())/((double)RAND_MAX);

    uint32_t rComponent=round(r*255.0);
    uint32_t gComponent=round(g*255.0);
    uint32_t bComponent=round(b*255.0);
    return 0xFF000000|(rComponent<<16)|(gComponent<<8)|bComponent;
}

int32_t MainWindow::round(double in)
{
    int32_t f=floor(in);
    return in<0.0?((double)in-f>0.5?f+1:f):((double)in-f>=0.5?f+1:f);
}

void MainWindow::browseButtonClicked()
{
    dialog->exec();
}

void MainWindow::fileSelected(QString file)
{
    currentFile=file;
    ui->lineEdit->setText(file.replace("/","\\"));
    dialog->setDirectory(QFileInfo(file).absoluteDir());

    QFile f(currentFile);
    if(!f.exists())
    {
        QMessageBox::critical(this,"Error","The selected file does not exist.");
        return;
    }
    delete originalImage;
    originalImage=new QImage(currentFile);
    if(originalImage->isNull())
    {
        QMessageBox::critical(this,"Error","The format of the selected file is not supported.");
        originalImage=0;
        return;
    }
    free(bmpData);
    bmpData=qImageToBitmapData(originalImage);
    delete filteredImage;
    filteredImage=0;
    delete areaColors;
    areaColors=0;
    free(areaRedirects);
    areaRedirects=0;

    scene->setSceneRect(0,0,originalImage->width(),originalImage->height());
    pixmapItem->setPixmap(QPixmap::fromImage(*originalImage));
    ui->graphicsView->viewport()->update();
    fitToWindow();
}

void MainWindow::groupAreasBtnClicked()
{
    if(originalImage==0)
        return;

    int32_t width=originalImage->width();
    int32_t height=originalImage->height();

    free(areaRedirects);
    delete areaColors;

    uint32_t areaCounter=0;
    uint32_t widthAndHeightBasedUInt32ArraySize=width*height*sizeof(uint32_t);
    areaRedirects=(uint32_t*)malloc(widthAndHeightBasedUInt32ArraySize);
    memset(areaRedirects,0xff,widthAndHeightBasedUInt32ArraySize); // Initialize with 0xffffffff
    areaColors=new std::map<uint32_t,uint32_t>();
    free(pixelAreas);
    pixelAreas=(uint32_t*)malloc(widthAndHeightBasedUInt32ArraySize);
    memset(pixelAreas,0xff,widthAndHeightBasedUInt32ArraySize); // Initialize with 0xffffffff

    // Create color groups

    double tolerance=ui->colorGroupingToleranceBox->value();
    int32_t kernelSize=ui->colorGroupingKernelSizeBox->value();
    int32_t kernelSizeInPixels=2*kernelSize+1;
    double sqrtOf3=sqrt(3.0);

    double *rChannel;
    double *gChannel;
    double *bChannel;
    getRGBChannelsFromImage(bmpData,width,height,rChannel,gChannel,bChannel);

    for(int32_t y=0;y<height;y++)
    {
        for(int32_t x=0;x<width;x++)
        {
            // We want to add this pixel to the group of the pixel in the surrounding with the lowest error (if the error is not above the treshold)

            uint32_t thisPos=y*width+x;
            uint32_t thisArea=pixelAreas[thisPos];

            // Resolve redirects

            if(thisArea!=0xffffffff)
            {
                uint32_t nextRedirect;
                while((nextRedirect=areaRedirects[thisArea])!=0xffffffff)
                    thisArea=pixelAreas[thisPos]=nextRedirect;
            }

            double lowestError=std::numeric_limits<double>::max();
            uint32_t lowestErrorArea=0xffffffff;
            int32_t lowestErrorX=-1;
            int32_t lowestErrorY=-1;

            double thisR=rChannel[thisPos];
            double thisG=gChannel[thisPos];
            double thisB=bChannel[thisPos];

            for(int32_t yOfKernel=0;yOfKernel<kernelSizeInPixels;yOfKernel++)
            {
                for(int32_t xOfKernel=0;xOfKernel<kernelSizeInPixels;xOfKernel++)
                {
                    // Check whether this pixel exists and whether it is not the same pixel

                    int32_t xWithKernel=-kernelSize+x+xOfKernel;
                    int32_t yWithKernel=-kernelSize+y+yOfKernel;

                    if(xWithKernel<0||xWithKernel>=width||
                       yWithKernel<0||yWithKernel>=height||
                       (xWithKernel==x&&yWithKernel==y))
                        continue;

                    uint32_t cmpPos=yWithKernel*width+xWithKernel;
                    uint32_t cmpArea=pixelAreas[cmpPos];

                    // Resolve redirects

                    if(cmpArea!=0xffffffff)
                    {
                        uint32_t nextCmpRedirect;
                        while((nextCmpRedirect=areaRedirects[cmpArea])!=0xffffffff)
                            cmpArea=pixelAreas[cmpPos]=nextCmpRedirect;
                    }

                    // If areas already identical

                    if(thisArea!=0xffffffff&&cmpArea==thisArea)
                        continue;

                    double cmpR=rChannel[cmpPos];
                    double cmpG=gChannel[cmpPos];
                    double cmpB=bChannel[cmpPos];

                    double error=sqrt(pow(thisR-cmpR,2.0)+pow(thisG-cmpG,2.0)+pow(thisB-cmpB,2.0))/sqrtOf3; // Max result: 1.0; max error term: sqrt(3.0)
                    if(error<lowestError)
                    {
                        lowestError=error;
                        lowestErrorArea=cmpArea;
                        lowestErrorX=xWithKernel;
                        lowestErrorY=yWithKernel;
                    }
                }
            }

            if(lowestError<=tolerance)
            {
                // Merge areas
                // Set area of every pixel in old area, including this pixel, to new area
                if(lowestErrorArea==0xffffffff)
                {
                    if(thisArea==0xffffffff)
                    {
                        // None of the pixels has an area; create new area

                        uint32_t newArea=areaCounter++;
                        areaColors->insert(std::pair<uint32_t,uint32_t>(newArea,getRandomColor()));
                        pixelAreas[thisPos]=newArea;
                        pixelAreas[lowestErrorY*width+lowestErrorX]=newArea;
                    }
                    else
                    {
                        // This pixel has an area
                        pixelAreas[lowestErrorY*width+lowestErrorX]=thisArea;
                    }
                }
                else if(thisArea==0xffffffff)
                {
                    // Lowest error pixel has an area
                    pixelAreas[thisPos]=lowestErrorArea;
                }
                else // Both pixels have an area; redirect this pixel's area to the area of the pixel with the lowest error.
                    areaRedirects[thisArea]=lowestErrorArea;
            }
        }
    }

    // Visualize areas

    uint32_t *outputImageData=(uint32_t*)malloc(width*height*sizeof(uint32_t));
    for(int32_t y=0;y<height;y++)
    {
        for(int32_t x=0;x<width;x++)
        {
            uint32_t pos=y*width+x;
            uint32_t area=pixelAreas[pos];
            if(area==0xffffffff)
                outputImageData[pos]=getRandomColor();
            else
            {
                // Resolve until no redirect left
                uint32_t nextRedirect;
                while((nextRedirect=areaRedirects[area])!=0xffffffff)
                    area=pixelAreas[pos]=nextRedirect;
                outputImageData[pos]=areaColors->at(area);
            }
        }
    }

    delete filteredImage;
    filteredImage=getImageFromBmpData(width,height,outputImageData);
    pixmapItem->setPixmap(QPixmap::fromImage(*filteredImage));
    free(outputImageData);
    free(rChannel);
    free(gChannel);
    free(bChannel);
}

void MainWindow::resetBtnClicked()
{
    if(originalImage==0)
        return;
    pixmapItem->setPixmap(QPixmap::fromImage(*originalImage));
    delete filteredImage;
    filteredImage=0;
}

void MainWindow::fitToWindow()
{
    if(originalImage==0||originalImage->isNull())
        return;
    int width=originalImage->width();
    int height=originalImage->height();
    QRect rect=ui->graphicsView->contentsRect();
    int availableWidth=rect.width()-ui->graphicsView->verticalScrollBar()->width();
    int availableHeight=rect.height()-ui->graphicsView->horizontalScrollBar()->height();
    if((width-availableWidth)>(height-availableHeight))
        ui->graphicsView->setZoomFactor((float)((float)availableWidth)/((float)width));
    else if(height>availableHeight)
        ui->graphicsView->setZoomFactor((float)((float)availableHeight)/((float)height));
    else
        ui->graphicsView->setZoomFactor(1.0);
}

void MainWindow::resetZoom()
{
    ui->graphicsView->setZoomFactor(1.0);
}

Point::Point()
{
    x=0;
    y=0;
}

Point::Point(int32_t _x, int32_t _y)
{
    x=_x;
    y=_y;
}
