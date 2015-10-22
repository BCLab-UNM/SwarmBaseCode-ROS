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
    painter.setRenderHint(QPainter::Antialiasing, true);

     tuple<float, float, float> eye = make_tuple(0, 0, 0);
     tuple<float, float, float> camera_position = make_tuple(0, 50, 0);
     tuple<float, float, float> camera_angle = make_tuple(0, 0, 0);


    tuple<float, float, float> points[4];
    points[0] = make_tuple(50, 50, 0);
    points[1] = make_tuple(50, 100, 0);
    points[2] = make_tuple(100, 100, 0);
    points[3] = make_tuple(100, 50, 0);


    QPoint projected_points[4];

    for (int i = 0; i < 4; i++)
    {
        projected_points[i] = cameraTransform(points[i], eye, camera_position, camera_angle);
    }

    painter.drawPolygon(projected_points,4);

    painter.drawText(50, 50, "IMU");
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

QPoint IMUFrame::cameraTransform( tuple<float, float, float> point_3D, tuple<float, float, float> eye, tuple<float, float, float> camera_position, tuple<float, float, float> camera_angle )
{
    tuple<float, float, float> a = point_3D;
    tuple<float, float, float> cam = camera_position;
    tuple<float, float, float> theta = camera_angle;

    float x = (get<0>(a)-get<0>(cam));
    float y = (get<1>(a)-get<1>(cam));
    float z = (get<2>(a)-get<2>(cam));
    float c_x = cos(get<0>(theta));
    float c_y = cos(get<1>(theta));
    float c_z = cos(get<2>(theta));
    float s_x = sin(get<0>(theta));
    float s_y = sin(get<1>(theta));
    float s_z = sin(get<2>(theta));
    float e_x = get<0>(eye);
    float e_y = get<1>(eye);
    float e_z = get<2>(eye);

    float d_x = c_y*(s_z*y+c_z*x)-s_y*z;
    float d_y = s_x*(c_y*z+s_y*(s_z*y+c_z*x))+c_x*(c_z*y-s_z*x);
    float d_z = c_x*(c_y*z+s_y*(s_z*y+c_z*x))-s_x*(c_z*y-s_z*x);

    // Transformation to b
    float b_x = (e_x/d_z)*d_x-e_x;
    float b_y = (e_z/d_z)*d_y-e_y;

    return QPoint(b_x, b_y);
}

}
#endif
