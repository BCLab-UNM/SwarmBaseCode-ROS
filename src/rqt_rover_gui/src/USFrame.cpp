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
    left_range = 1.0;
    right_range = 1.0;
    center_range = 1.0;
    left_max_range = 1.0;
    right_max_range = 1.0;
    center_max_range = 1.0;
    left_min_range = 0.0;
    right_min_range = 0.0;
    center_min_range = 0.0;

}

void USFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);


    float frame_width = this->width();
    float frame_height = this->height();

    // Use unit coordinate system and scale to the size of the frame

    float frame_center_x = frame_width*0.5;
    float frame_center_y = frame_height*0.5;


    QPoint left_end_point(0, 0);
    QPoint right_end_point(frame_width, 0);
    QPoint center_end_point(frame_center_x, 0);

    QPoint start_point(frame_width/2, frame_height-20);

    float left_scale = left_range/(left_max_range-left_min_range);
    float right_scale = right_range/(right_max_range-right_min_range);
    float center_scale = center_range/(center_max_range-center_min_range);

    // Equation of a line from two points. The cooefficent (*_scale) scales the line.
    // Presumes the range is on the interval [0,1]
    QPoint left_vector = start_point + left_scale*(left_end_point-start_point);
    QPoint right_vector = start_point + right_scale*(right_end_point-start_point);
    QPoint center_vector = start_point + center_scale*(center_end_point-start_point);

    painter.drawLine(start_point, left_vector);
    painter.drawLine(start_point, center_vector);
    painter.drawLine(start_point, right_vector);


    float left_range_rounded = roundf(left_range * 100) / 100;
    float right_range_rounded = roundf(right_range * 100) / 100;
    float center_range_rounded = roundf(center_range * 100) / 100;


    QString left_range_in_meters_qstr = QString::number(left_range_rounded*3)+"m";
    QString right_range_in_meters_qstr = QString::number(right_range_rounded*3)+"m";
    QString center_range_in_meters_qstr = QString::number(center_range_rounded*3)+"m";

    QString left_range_qstr = QString::number(left_range_rounded);
    QString right_range_qstr = QString::number(right_range_rounded);
    QString center_range_qstr = QString::number(center_range_rounded);


    QFontMetrics fm(painter.font());

    painter.drawText(QPoint(frame_center_x-frame_width/4-fm.width(left_range_in_meters_qstr)/2,frame_height), left_range_in_meters_qstr);
    painter.drawText(QPoint(frame_center_x-fm.width(right_range_in_meters_qstr)/2,frame_height), right_range_in_meters_qstr);
    painter.drawText(QPoint(frame_center_x+frame_width/4-fm.width(right_range_in_meters_qstr)/2,frame_height), center_range_in_meters_qstr);

}

void USFrame::setCenterRange(float r, float min, float max)
{
    center_range = r;
    center_min_range = min;
    center_max_range = max;
    emit delayedUpdate();
}

void USFrame::setLeftRange(float r, float min, float max)
{
    left_range = r;
    left_min_range = min;
    left_max_range = max;
    emit delayedUpdate();
}

void USFrame::setRightRange(float r, float min, float max)
{
    right_range = r;
    right_min_range = min;
    right_max_range = max;
    emit delayedUpdate();
}

}
#endif
