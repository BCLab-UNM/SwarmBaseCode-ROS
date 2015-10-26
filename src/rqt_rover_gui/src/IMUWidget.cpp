#ifndef rqt_rover_gui_IMUWidget
#define rqt_rover_gui_IMUWidget

#include <iostream>
#include <cmath>



#include <IMUWidget.h>

namespace rqt_rover_gui
{

IMUWidget::IMUWidget(QWidget *parent, Qt::WFlags flags) : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

    linear_acceleration = make_tuple(0,0,0); // ROS Geometry Messages Vector3: <x, y, z> -- Initialize to all 0s
    angular_velocity = make_tuple(0,0,0); // ROS Geometry Messages Vector3: <x, y, z>    -- Initialize to all 0s
    orientation = make_tuple(0,0,0,0); // ROS Geometry Messages Quaternion: <w, x, y, z>   -- Initialize to all 0s

}

void IMUWidget::paintGL()
{
}

void IMUWidget::setLinearAcceleration(float x, float y, float z)
{
    linear_acceleration = make_tuple(x, y, z);
}

void IMUWidget::setAngularVelocity(float x, float y, float z)
{
    angular_velocity = make_tuple(x, y, z);
}

void IMUWidget::setOrientation(float w, float x, float y, float z)
{
    orientation = make_tuple(w, x, y, z);
}

void IMUWidget::initializeGL()
{
//    char *my_argv[] = { "IMUWidget", NULL };
//    int   my_argc = 1;
//    glShadeModel(GL_SMOOTH);
//    glEnable(GL_MULTISAMPLE);
//    glEnable(GL_DEPTH_CLAMP);
}

}
#endif
