#ifndef rqt_rover_gui_IMUFrame
#define rqt_rover_gui_IMUFrame

#include <QTimer>
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

        // Make a test cube
        float center_x = this->width()/2;
        float center_y = this->height()/2;
        float width_of_square = 100;

        cube[0] = make_tuple(width_of_square/2, -width_of_square/2, -width_of_square/2);
        cube[1] = make_tuple(width_of_square/2, width_of_square/2, -width_of_square/2);
        cube[2] = make_tuple(-width_of_square/2, width_of_square/2, -width_of_square/2);
        cube[3] = make_tuple(-width_of_square/2, -width_of_square/2, -width_of_square/2);
        cube[4] = make_tuple(width_of_square/2, -width_of_square/2, width_of_square/2);
        cube[5] = make_tuple(width_of_square/2, width_of_square/2, width_of_square/2);
        cube[6] = make_tuple(-width_of_square/2, width_of_square/2, width_of_square/2);
        cube[7] = make_tuple(-width_of_square/2, -width_of_square/2, width_of_square/2);


        // Setup a timer to rotate the square every 1/10 second
        QTimer *timer = new QTimer(this);
           connect(timer, SIGNAL(timeout()), this, SLOT(rotateTimerEventHandler()));
           timer->start(100);
}

void IMUFrame::rotateTimerEventHandler()
{
    float center_x = this->width()/2;
    float center_y = this->height()/2;

    tuple<float,float,float> axis_of_rotation = make_tuple(rand()%20,rand()%20,rand()%20);

    for (int i = 0; i < 8; i++)
        cube[i] = rotateAboutAxis(cube[i], M_PI/10, axis_of_rotation);


    emit delayedUpdate();
}

void IMUFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);
    painter.setRenderHint(QPainter::Antialiasing, true);

    float center_x = this->width()/2;

    float center_y = this->height()/2;

    // Setup axes
    tuple<float, float, float> axes_origin = make_tuple(0,0,0);
    tuple<float, float, float> x_axis = make_tuple(this->width(),0,0);
    tuple<float, float, float> y_axis = make_tuple(0,this->height(),0);
    tuple<float, float, float> z_axis = make_tuple(0,0,this->width());


    // Setup camera transform inputs
     tuple<float, float, float> eye = make_tuple(0, 0, 1000);
     tuple<float, float, float> camera_position = make_tuple(0, 0, 1080);
     tuple<float, float, float> camera_angle = make_tuple(0, 0, 0);

    // Project 3D points into 2D
    QPoint projected_cube[8];
    QPoint projected_axes_origin;
    QPoint projected_x_axis;
    QPoint projected_y_axis;
    QPoint projected_z_axis;

    for (int i = 0; i < 8; i++)
    {
        projected_cube[i] = cameraTransform(cube[i], eye, camera_position, camera_angle);
        projected_axes_origin = cameraTransform(axes_origin, eye, camera_position, camera_angle);
        projected_x_axis = cameraTransform(x_axis, eye, camera_position, camera_angle);
        projected_y_axis = cameraTransform(y_axis, eye, camera_position, camera_angle);
        projected_z_axis = cameraTransform(z_axis, eye, camera_position, camera_angle);
    }

    // Translate point positions into the desired positions in the frame
    for (int i = 0; i < 8; i++)
    {
        projected_cube[i].setX(projected_cube[i].x()+center_x);
        projected_cube[i].setY(projected_cube[i].y()+center_y);
    }

    projected_axes_origin.setX(projected_axes_origin.x()+this->width());
    projected_axes_origin.setY(projected_axes_origin.y()+this->height());

    projected_x_axis.setX(projected_x_axis.x()+this->width());
    projected_x_axis.setY(projected_x_axis.y()+this->height());

    projected_y_axis.setX(projected_y_axis.x()+this->width());
    projected_y_axis.setY(projected_y_axis.y()+this->height());

    projected_z_axis.setX(projected_z_axis.x()+this->width());
    projected_z_axis.setY(projected_z_axis.y()+this->height());


    // Draw lines connecting the projected points

    // Draw the axes
    painter.setPen(Qt::red);
    painter.drawLine(projected_axes_origin, projected_x_axis);

    painter.setPen(Qt::blue);
    painter.drawLine(projected_axes_origin, projected_y_axis);

    painter.setPen(Qt::yellow);
    painter.drawLine(projected_axes_origin, projected_z_axis);

    painter.setPen(Qt::white);

    // Draw the cube
    QPoint projected_cube_top[4];
    QPoint projected_cube_bottom[4];

    projected_cube_top[0] = projected_cube[0];
    projected_cube_top[1] = projected_cube[1];
    projected_cube_top[2] = projected_cube[2];
    projected_cube_top[3] = projected_cube[3];

    projected_cube_bottom[0] = projected_cube[4];
    projected_cube_bottom[1] = projected_cube[5];
    projected_cube_bottom[2] = projected_cube[6];
    projected_cube_bottom[3] = projected_cube[7];

    painter.drawLine(projected_cube_top[0], projected_cube_bottom[0]);
    painter.drawLine(projected_cube_top[1], projected_cube_bottom[1]);
    painter.drawLine(projected_cube_top[2], projected_cube_bottom[2]);
    painter.drawLine(projected_cube_top[3], projected_cube_bottom[3]);
    painter.drawPolygon(projected_cube_top,4);
    painter.drawPolygon(projected_cube_bottom,4);



 //   painter.drawText(50, 50, "IMU");
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

    float b_x = (e_z/d_z)*d_x-e_x;
    float b_y = (e_z/d_z)*d_y-e_y;

    return QPoint(b_x, b_y);
}


float** IMUFrame::setUpRotationMatrix(float angle, tuple<float, float, float> axis_of_rotation) // angle in radians
{
    float u = get<0>(axis_of_rotation);
    float v = get<1>(axis_of_rotation);
    float w = get<2>(axis_of_rotation);

    float L = (u*u + v*v + w*w);
    float u2 = u*u;
    float v2 = v*v;
    float w2 = w*w;

    float** rotation_matrix = new float*[4];
    for (int i = 0; i < 4; i++)
        rotation_matrix[i] = new float[4];

    rotation_matrix[0][0] = (u2 + (v2 + w2) * cos(angle)) / L;
    rotation_matrix[0][1] = (u * v * (1 - cos(angle)) - w * sqrt(L) * sin(angle)) / L;
    rotation_matrix[0][2] = (u * w * (1 - cos(angle)) + v * sqrt(L) * sin(angle)) / L;
    rotation_matrix[0][3] = 0.0;

    rotation_matrix[1][0] = (u * v * (1 - cos(angle)) + w * sqrt(L) * sin(angle)) / L;
    rotation_matrix[1][1] = (v2 + (u2 + w2) * cos(angle)) / L;
    rotation_matrix[1][2] = (v * w * (1 - cos(angle)) - u * sqrt(L) * sin(angle)) / L;
    rotation_matrix[1][3] = 0.0;

    rotation_matrix[2][0] = (u * w * (1 - cos(angle)) - v * sqrt(L) * sin(angle)) / L;
    rotation_matrix[2][1] = (v * w * (1 - cos(angle)) + u * sqrt(L) * sin(angle)) / L;
    rotation_matrix[2][2] = (w2 + (u2 + v2) * cos(angle)) / L;
    rotation_matrix[2][3] = 0.0;

    rotation_matrix[3][0] = 0.0;
    rotation_matrix[3][1] = 0.0;
    rotation_matrix[3][2] = 0.0;
    rotation_matrix[3][3] = 1.0;

    return rotation_matrix;
}

    tuple<float, float, float> IMUFrame::rotateAboutAxis(tuple<float, float, float> point, float angle, tuple<float, float, float> axis_of_rotation )
    {
       float** rotation_matrix = setUpRotationMatrix(angle, axis_of_rotation);

       tuple<float, float, float> rotated_point;

       float position_vector[4] = {0.0, 0.0, 0.0, 0.0};
       float rotated_position_vector[4] = {0.0, 0.0, 0.0, 0.0};

       position_vector[0] = get<0>(point);
       position_vector[1] = get<1>(point);
       position_vector[2] = get<2>(point);
       position_vector[3] = 1.0;

       // Multiply the position vector by the rotation matrix
        for(int i = 0; i < 4; i++ )
            for(int k = 0; k < 4; k++)
                rotated_position_vector[i] += rotation_matrix[i][k] * position_vector[k];

      // Clean up the rotation matrix
      for (int i = 0; i < 4; i++) delete [] rotation_matrix[i];
      delete [] rotation_matrix;

      get<0>(rotated_point) = rotated_position_vector[0];
      get<1>(rotated_point) = rotated_position_vector[1];
      get<2>(rotated_point) = rotated_position_vector[2];

      return rotated_point;

    }


}
#endif
