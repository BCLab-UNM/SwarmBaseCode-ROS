#ifndef rqt_rover_gui_GPSFrame
#define rqt_rover_gui_GPSFrame

#include <iostream>
#include <cmath>

#include <GPSFrame.h>

namespace rqt_rover_gui
{

GPSFrame::GPSFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

        frames = 0;
}

void GPSFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);

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

    painter.drawText(QPoint(50,50), "GPS Frame");

    painter.setPen(Qt::white);
}

}
#endif
