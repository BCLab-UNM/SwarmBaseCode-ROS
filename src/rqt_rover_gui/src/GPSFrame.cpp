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

}

void GPSFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);

    painter.drawText(QPoint(50,50), "GPS Frame");

    painter.setPen(Qt::white);
}

}
#endif
