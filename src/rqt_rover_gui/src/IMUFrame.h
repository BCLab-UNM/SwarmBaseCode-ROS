/*!
 * \brief   This class visualizes the output from the rover's Inertial Measurement Unit (IMU).
 *          Currently the orientation and acceleration of the rover are displayed.
 *          The IMU sensor display consists of a cube where the red face is the bottom of the rover,
 *          the blue face is the top of the rover, and the red and blue bars are the front and back of the rover.
 *          The cube is viewed from the top down. The cube is positioned according to the IMU orientation data.
 *          For example, if the rover flips over, the red side will be closest to the observer.
 *          Accelerometer data is shown as a 3D vector projected into 2D space pointing towards the sum of the accelerations in 3D space.
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo
 * \class   IMUFrame
 */

#ifndef IMUFRAME_H
#define IMUFRAME_H

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector> // For standard template library vectors
#include <tuple> //  For standard template library tuples
#include <utility> // For STL pair

using namespace std;

// ROS rqt requires gui elements be in the UI plugin namespace
namespace rqt_rover_gui
{

class IMUFrame : public QFrame
{
    Q_OBJECT
public:
    IMUFrame(QWidget *parent, Qt::WindowFlags = 0);
    void setLinearAcceleration(float x, float y, float z);
    void setAngularVelocity(float x, float y, float z);
    void setOrientation(float w, float x, float y, float z);

signals:

    void delayedUpdate();

public slots:
    void rotateTimerEventHandler();


protected:

    void paintEvent(QPaintEvent *event);

private:
    QPoint cameraTransform( tuple<float, float, float> point_3D, tuple<float, float, float> eye, tuple<float, float, float> camera_position, tuple<float, float, float> camera_angle );
    tuple<float, float, float> rotateByQuaternion( tuple<float, float, float> point,  tuple<float, float, float, float> quaternion);
    tuple<float, float, float> inverseRotateByQuaternion( tuple<float, float, float> point,  tuple<float, float, float, float> quaternion);
    tuple<float, float, float> rotateAboutAxis(tuple<float, float, float> point, float angle,  tuple<float, float, float> axis_of_rotation);
    float** setUpRotationMatrix(float angle, tuple<float, float, float> axis_rotation); // angle in radians   

    tuple<float, float, float> linear_acceleration; // ROS Geometry Messages Vector3: <x, y, z>
    tuple<float, float, float> angular_velocity; // ROS Geometry Messages Vector3: <x, y, z>
    tuple<float, float, float, float> orientation; // ROS Geometry Messages Quaternion: <w, x, y, z>

    // Test points to render
    tuple<float, float, float> cube[8];
    tuple<float, float, float> rotated_cube[8];

    tuple<float, float, float> line1_start;
    tuple<float, float, float> line1_end;

    tuple<float, float, float> line2_start;
    tuple<float, float, float> line2_end;

    tuple<float, float, float> rotated_line1_start;
    tuple<float, float, float> rotated_line1_end;

    tuple<float, float, float> rotated_line2_start;
    tuple<float, float, float> rotated_line2_end;

    QTime frame_rate_timer;
    int frames;
};

}

#endif // IMUFrame_H
