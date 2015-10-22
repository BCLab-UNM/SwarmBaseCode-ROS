#ifndef rqt_rover_gui_IMUFrame
#define rqt_rover_gui_IMUFrame

#include <iostream>
#include <cmath>

#include <IMUFrame.h>

namespace rqt_rover_gui
{

IMUFrame::IMUFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

    linear_acceleration = make_tuple(0,0,0); // ROS Geometry Messages Vector3: <x, y, z> -- Initialize to all 0s
    angular_velocity = make_tuple(0,0,0); // ROS Geometry Messages Vector3: <x, y, z>    -- Initialize to all 0s
    orientation = make_tuple(0,0,0,0); // ROS Geometry Messages Quaternion: <w, x, y, z>   -- Initialize to all 0s

}

void IMUFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);

    painter.drawText(QPoint(10,50), "Angular Velocity: <" + QString::number(get<0>(angular_velocity)) + ", "
                                                       + "<" + QString::number(get<1>(angular_velocity)) + ", "
                                                       + "<" + QString::number(get<2>(angular_velocity)) + ">" );

    painter.drawText(QPoint(10,70), "Linear Acceleration: <" + QString::number(get<0>(linear_acceleration)) + ", "
                                                       + "<" + QString::number(get<1>(linear_acceleration)) + ", "
                                                       + "<" + QString::number(get<2>(linear_acceleration)) + ">" );

    painter.drawText(QPoint(10,90), "Orienation: <" + QString::number(get<0>(orientation)) + ", "
                                                       + "<" + QString::number(get<1>(orientation)) + ", "
                                                       + "<" + QString::number(get<2>(orientation)) + ", "
                                                       + "<" + QString::number(get<2>(orientation)) + ">" );


    painter.setPen(Qt::white);
}

void IMUFrame::setLinearAcceleration(float x, float y, float z)
{
    linear_acceleration = make_tuple(x, y, z);
}

void IMUFrame::setAngularVelocity(float x, float y, float z)
{
    angular_velocity = make_tuple(x, y, z);
}

void IMUFrame::setOrientation(float w, float x, float y, float z)
{
    orientation = make_tuple(w, x, y, z);
}

}
#endif
