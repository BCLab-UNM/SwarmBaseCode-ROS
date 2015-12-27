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

        line1_start = make_tuple(width_of_square/2, 0, 0);
        line2_start = make_tuple(-width_of_square/2, 0, 0);

        line1_end = make_tuple(width_of_square, 0, 0);
        line2_end = make_tuple(-width_of_square, 0, 0);

        // Setup a timer to rotate the square every 1/10 second
//        QTimer *timer = new QTimer(this);
//        connect(timer, SIGNAL(timeout()), this, SLOT(rotateTimerEventHandler()));
//        timer->start(100);

        //  axis/angle rotation (a,x,y,z) is equal to quaternion (cos(a/2),xsin(a/2),ysin(a/2),z*sin(a/2)) if you want to use the rotateAboutAxis function

        // Do this because IMU data is reversed in the Z direction
        // Angle of rotation
        float angle = 180 *M_PI/180.0f;

        // Axis of rotation
        float x = 0;
        float y = 3;
        float z = 0;

        float mag = sqrt(x*x+y*y+z*z);

        tuple<float,float,float,float> quaternion = make_tuple(cos(angle/2),x*sin(angle/2)/mag,y*sin(angle/2)/mag,z*sin(angle/2)/mag);
        for (int i = 0; i < 8; i++)
        cube[i] = inverseRotateByQuaternion(cube[i], quaternion);

        // Initialize the rotated_cube
        for (int i = 0; i < 8; i++)
        rotated_cube[i] = cube[i];

        // Rotate the lines coming out of the cube
        line1_start = inverseRotateByQuaternion(line1_start, quaternion);
        rotated_line2_start = inverseRotateByQuaternion(line2_start, quaternion);

        line1_end = inverseRotateByQuaternion(line1_end, quaternion);
        line2_end = inverseRotateByQuaternion(line2_end, quaternion);

        rotated_line1_start = line1_start;
        rotated_line2_start = line2_start;

        rotated_line1_end = line1_end;
        rotated_line1_end = line1_end;

        frames = 0;
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

    // Setup axes
    tuple<float, float, float> axes_origin = make_tuple(0,0,0);
    tuple<float, float, float> x_axis = make_tuple(this->width(),0,0);
    tuple<float, float, float> y_axis = make_tuple(0,this->height(),0);
    tuple<float, float, float> z_axis = make_tuple(0,0,this->width());


    // Setup camera transform inputs
     tuple<float, float, float> eye = make_tuple(0, 0, 1000);
     tuple<float, float, float> camera_position = make_tuple(0, 0, 1080);
     tuple<float, float, float> camera_angle = make_tuple(0, 0, M_PI/2);

    // Project 3D points into 2D
    QPoint projected_cube[8];
    QPoint projected_axes_origin;
    QPoint projected_x_axis;
    QPoint projected_y_axis;
    QPoint projected_z_axis;

    for (int i = 0; i < 8; i++)
    {
        projected_cube[i] = cameraTransform(rotated_cube[i], eye, camera_position, camera_angle);
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

    // Colorblind friendly colors
    QColor green(17, 192, 131);
    QColor red(255, 65, 30);

    painter.setPen(Qt::red);
    painter.drawLine(projected_axes_origin, projected_x_axis);

    painter.setPen(Qt::blue);
    painter.drawLine(projected_axes_origin, projected_y_axis);

    painter.setPen(Qt::yellow);
    painter.drawLine(projected_axes_origin, projected_z_axis);

    painter.setPen(Qt::white);

    // draw two lines orthogonal to the yz faces of the cube to help visualize orientation
    QPoint projected_line1_start = cameraTransform(rotated_line1_start, eye, camera_position, camera_angle);
    QPoint projected_line1_end = cameraTransform(rotated_line1_end, eye, camera_position, camera_angle);
    QPoint projected_line2_start = cameraTransform(rotated_line2_start, eye, camera_position, camera_angle);
    QPoint projected_line2_end = cameraTransform(rotated_line2_end, eye, camera_position, camera_angle);

    // Translate to the right place in the frame
    projected_line1_start.setX(projected_line1_start.x()+center_x);
    projected_line1_start.setY(projected_line1_start.y()+center_y);

    projected_line1_end.setX(projected_line1_end.x()+center_x);
    projected_line1_end.setY(projected_line1_end.y()+center_y);

    projected_line2_start.setX(projected_line2_start.x()+center_x);
    projected_line2_start.setY(projected_line2_start.y()+center_y);

    projected_line2_end.setX(projected_line2_end.x()+center_x);
    projected_line2_end.setY(projected_line2_end.y()+center_y);

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

    QPainterPath top_path;
    top_path.moveTo(projected_cube_top[0]);
    top_path.lineTo(projected_cube_top[1]);
    top_path.lineTo(projected_cube_top[2]);
    top_path.lineTo(projected_cube_top[3]);
    top_path.lineTo(projected_cube_top[0]);

    QPainterPath bottom_path;
    bottom_path.moveTo(projected_cube_bottom[0]);
    bottom_path.lineTo(projected_cube_bottom[1]);
    bottom_path.lineTo(projected_cube_bottom[2]);
    bottom_path.lineTo(projected_cube_bottom[3]);
    bottom_path.lineTo(projected_cube_bottom[0]);

    // Draw top and bottom faces with the nearest drawn on top (i.e. last)
    tuple<float,float,float> top_corner = rotated_cube[0];
    tuple<float,float,float> bottom_corner = rotated_cube[7];
    float bottom_z = get<2>(bottom_corner);
    float top_z = get<2>(top_corner);

    if (top_z < bottom_z)
    {
        painter.drawPolygon(projected_cube_top,4);
        painter.fillPath(top_path,Qt::blue);
        painter.setPen(Qt::blue);
        painter.drawLine(projected_line1_start, projected_line1_end);
        painter.setPen(red);
        painter.drawLine(projected_line2_start, projected_line2_end);
        painter.setPen(Qt::white);
        painter.drawPolygon(projected_cube_bottom,4);
        painter.fillPath(bottom_path,red);
    }
    else
    {
        painter.drawPolygon(projected_cube_bottom,4);
        painter.fillPath(bottom_path,Qt::red);
        painter.setPen(Qt::blue);
        painter.drawLine(projected_line1_start, projected_line1_end);
        painter.setPen(Qt::red);
        painter.drawLine(projected_line2_start, projected_line2_end);
        painter.setPen(Qt::white);
        painter.drawPolygon(projected_cube_top,4);
        painter.fillPath(top_path,Qt::blue);
    }


    // Draw wireframe on top
    painter.drawLine(projected_cube_top[0], projected_cube_bottom[0]);
    painter.drawLine(projected_cube_top[1], projected_cube_bottom[1]);
    painter.drawLine(projected_cube_top[2], projected_cube_bottom[2]);
    painter.drawLine(projected_cube_top[3], projected_cube_bottom[3]);




// Draw an acceleration arrow from the IMU accelerometer

tuple<float, float, float> accel_start = make_tuple(0,0,0);
tuple<float, float, float> accel_end = linear_acceleration;

tuple<float, float, float> accel_end_head_x_left = accel_end;
tuple<float, float, float> accel_end_head_x_right = accel_end;
get<0>(accel_end_head_x_left) = get<0>(accel_end_head_x_left)-1;
get<0>(accel_end_head_x_right) = get<0>(accel_end_head_x_right)+1;
get<1>(accel_end_head_x_left) = get<1>(accel_end_head_x_left)+1;
get<1>(accel_end_head_x_right) = get<1>(accel_end_head_x_right)+1;

// rotate about the x axis so z is up and down on the screen
tuple<float,float,float> axis_of_rotation = make_tuple(0,1,0);
accel_start = rotateAboutAxis(accel_start, M_PI/2, axis_of_rotation);
accel_end = rotateAboutAxis(accel_end, M_PI/2, axis_of_rotation);

accel_end_head_x_left = rotateAboutAxis(accel_end_head_x_left, M_PI/2, axis_of_rotation);
accel_end_head_x_right = rotateAboutAxis(accel_end_head_x_right, M_PI/2, axis_of_rotation);

QPoint projected_accel_start = cameraTransform(accel_start, eye, camera_position, camera_angle);
QPoint projected_accel_end = cameraTransform(accel_end, eye, camera_position, camera_angle);

QPoint projected_accel_end_head_x_left = cameraTransform(accel_end_head_x_left, eye, camera_position, camera_angle);
QPoint projected_accel_end_head_x_right = cameraTransform(accel_end_head_x_right, eye, camera_position, camera_angle);

painter.drawLine( QPoint(projected_accel_start.x()+center_x, projected_accel_start.y()+center_y),
                         QPoint(10*projected_accel_end.x()+center_x, 10*projected_accel_end.y()+center_y));

painter.drawLine( QPoint(projected_accel_end_head_x_left.x()+center_x, projected_accel_end_head_x_left.y()+center_y),
                                 QPoint(10*projected_accel_end.x()+center_x, 10*projected_accel_end.y()+center_y));

painter.drawLine( QPoint(projected_accel_end_head_x_right.x()+center_x, projected_accel_end_head_x_right.y()+center_y),
                                 QPoint(10*projected_accel_end.x()+center_x, 10*projected_accel_end.y()+center_y));





//    QPainterPath arrow_path;

//    arrow_path.moveTo(QPoint(25,10));
//    arrow_path.lineTo(QPoint(25,50));
//    arrow_path.lineTo(QPoint(35,50));
//    arrow_path.lineTo(QPoint(20,60));
//    arrow_path.lineTo(QPoint(5,50));
//    arrow_path.lineTo(QPoint(15,50));
//    arrow_path.lineTo(QPoint(15,10));
//    arrow_path.lineTo(QPoint(25,10));


//    painter.drawPath(arrow_path);


}

void IMUFrame::setLinearAcceleration(float x, float y, float z)
{
    linear_acceleration = make_tuple(x, y, z);
    emit delayedUpdate();
}

void IMUFrame::setAngularVelocity(float x, float y, float z)
{
    angular_velocity = make_tuple(x, y, z);
    emit delayedUpdate();
}

void IMUFrame::setOrientation(float w, float x, float y, float z)
{
    // Quaternions: A quaternion represents two things.  It has an x, y, and z component, which represents the axis about which a rotation will occur.
    // It also has a w component, which represents the amount of rotation which will occur about this axis. The rotationMatrix() function can use this representation
    // to rotate the object properly

//    orientation = make_tuple(w, x, y, z);
//    float angle_of_rotation = get<0>(orientation);
//    tuple<float, float, float> axis_of_rotation = make_tuple(get<1>(orientation), get<2>(orientation), get<3>(orientation));

//    for (int i = 0; i < 8; i++)
//        rotated_cube[i] = rotateAboutAxis(cube[i], angle_of_rotation, axis_of_rotation);

    tuple<float,float,float,float> quaternion = make_tuple(w,x,y,z);

    for (int i = 0; i < 8; i++)
        rotated_cube[i] = inverseRotateByQuaternion(cube[i], quaternion);

    rotated_line1_start = inverseRotateByQuaternion(line1_start, quaternion);
    rotated_line2_start = inverseRotateByQuaternion(line2_start, quaternion);

    rotated_line1_end = inverseRotateByQuaternion(line1_end, quaternion);
    rotated_line2_end = inverseRotateByQuaternion(line2_end, quaternion);


    emit delayedUpdate();
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

    tuple<float, float, float> IMUFrame::rotateByQuaternion(tuple<float, float, float> v, tuple<float, float, float, float>  quaternion)
    {

        tuple<float, float, float> rotated_point;

        // Extract the vector part of the quaternion
        //float magnitude = sqrt(get<1>(quaternion)*get<1>(quaternion)+get<2>(quaternion)*get<2>(quaternion)+get<3>(quaternion)*get<3>(quaternion));

        //cout << "Magnitude: " << magnitude << endl;

        //tuple<float, float, float> u = make_tuple(get<1>(quaternion)/magnitude, get<2>(quaternion)/magnitude, get<3>(quaternion)/magnitude);
        tuple<float, float, float> u = make_tuple(get<1>(quaternion), get<2>(quaternion), get<3>(quaternion));

        // Extract the scalar part of the quaternion
        float s = get<0>(quaternion);

        // Calculate rotated point
        // dot prod of u and v
        float dot_uv = get<0>(u)*get<0>(v)+get<1>(u)*get<1>(v)+get<2>(u)*get<2>(v);

        float dot_uu = get<0>(u)*get<0>(u)+get<1>(u)*get<1>(u)+get<2>(u)*get<2>(u);

        tuple<float, float, float> cross_uv = make_tuple(get<1>(u)*get<2>(v)-get<1>(v)*get<2>(u),
                                                         get<0>(v)*get<2>(u)-get<0>(u)*get<2>(v),
                                                         get<0>(u)*get<1>(v)-get<0>(v)*get<1>(u));

        rotated_point = make_tuple(2.0f * dot_uv * get<0>(u) + (s*s - dot_uu) * get<0>(v) + 2.0f * s * get<0>(cross_uv),
                                   2.0f * dot_uv * get<1>(u) + (s*s - dot_uu) * get<1>(v) + 2.0f * s * get<1>(cross_uv),
                                   2.0f * dot_uv * get<2>(u) + (s*s - dot_uu) * get<2>(v) + 2.0f * s * get<2>(cross_uv));

        return rotated_point;
    }

    tuple<float, float, float> IMUFrame::inverseRotateByQuaternion(tuple<float, float, float> v, tuple<float, float, float, float>  quaternion)
    {
        // Make the vector component of the quaternion negative. This reverses the rotation.
        return rotateByQuaternion(v,  make_tuple(get<0>(quaternion), -get<1>(quaternion), -get<2>(quaternion), -get<3>(quaternion)));
    }

}
#endif
