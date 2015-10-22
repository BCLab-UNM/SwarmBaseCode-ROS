#ifndef rqt_rover_gui_USFrame
#define rqt_rover_gui_USFrame

#include <iostream>
#include <cmath>

#include <USFrame.h>

namespace rqt_rover_gui
{

USFrame::USFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);
    left_range = 0;
    right_range = 0;
    center_range = 0;
}

void USFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);

    painter.drawText(QPoint(50,50), "Left Range: "+QString::number(left_range));
    painter.drawText(QPoint(50,70), "Center Range: "+QString::number(center_range));
    painter.drawText(QPoint(50,90), "Right Range: "+QString::number(right_range));

    painter.setPen(Qt::white);
}

void USFrame::setCenterRange(float r)
{
    center_range = r;
    emit delayedUpdate();
}

void USFrame::setLeftRange(float r)
{
    left_range = r;
    emit delayedUpdate();
}

void USFrame::setRightRange(float r)
{
    right_range = r;
    emit delayedUpdate();
}

}
#endif
