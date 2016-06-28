#include <CameraFrame.h>

namespace rqt_rover_gui
{

CameraFrame::CameraFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

        frames = 0;
}

void CameraFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);


    image_update_mutex.lock();
    if (!(image.isNull()))
    {
        //painter.drawText(QPoint(50,50), "Image Received From Camera");

        painter.drawImage(contentsRect(), image);

        QRect rect(target_c1[0], target_c1[1], target_c4[0] - target_c1[0], target_c4[1] - target_c1[1]);
        painter.drawRect(rect);

    }
    else
    {

        painter.drawText(QPoint(50,50), "No Image Received From Camera");
    }
    image_update_mutex.unlock();

    // Track the frames per second for development purposes
    QString frames_per_second;
    frames_per_second = QString::number(frames /(frame_rate_timer.elapsed() / 1000.0), 'f', 0) + " FPS";

    QFontMetrics fm(painter.font());
    painter.drawText(this->width()-fm.width(frames_per_second), fm.height(), frames_per_second);

     frames++;

    if (!(frames % 100)) // time how long it takes to dispay 100 frames
    {
        frame_rate_timer.start();
        frames = 0;
    }

    // end frames per second

}

void CameraFrame::setImage(const QImage& img)
{
    image_update_mutex.lock();
    image = img.copy();
    image_update_mutex.unlock();
    emit delayedUpdate();
}

void CameraFrame::addTarget(double c1[2], double c2[2], double c3[2], double c4[2])
{
    double *TL;
    double *BR;

    TL = BR = c1;

    (greaterThan(c2, c1)) ? BR = c2 : BR;
    (greaterThan(c3, BR)) ? BR = c3 : BR;
    (greaterThan(c4, BR)) ? BR = c4 : BR;

    (!greaterThan(c2, c1)) ? TL = c2 : TL;
    (!greaterThan(c3, TL)) ? TL = c3 : TL;
    (!greaterThan(c4, TL)) ? TL = c4 : TL;

    target_c1[0] = TL[0];
    target_c1[1] = TL[1];

    target_c4[0] = BR[0];
    target_c4[1] = BR[1];


}

bool greaterThan(double c1[2], double c2[2])
{
    double sum1 = c1[0] + c1[1];
    double sum2 = c2[0] + c2[1];

    return sum1 > sum1;
}

//bool greaterThan(double c1, double c2)
//{
    //return c1 > c2;
//}

}

