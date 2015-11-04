#ifndef rtq_rover_gui_IMUFrame_H
#define rtq_rover_gui_IMUFrame_H

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <utility> // For STL pair

using namespace std;

namespace rqt_rover_gui
{

class IMUFrame : public QFrame
{
    Q_OBJECT
public:
    IMUFrame(QWidget *parent, Qt::WFlags = 0);
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
    tuple<float, float, float> rotateAboutAxis(tuple<float, float, float> point, float angle,  tuple<float, float, float> axis_of_rotation);
    float** setUpRotationMatrix(float angle, tuple<float, float, float> axis_rotation); // angle in radians   

    tuple<float, float, float> linear_acceleration; // ROS Geometry Messages Vector3: <x, y, z>
    tuple<float, float, float> angular_velocity; // ROS Geometry Messages Vector3: <x, y, z>
    tuple<float, float, float, float> orientation; // ROS Geometry Messages Quaternion: <w, x, y, z>

    // Test points to render
    tuple<float, float, float> cube[8];
    tuple<float, float, float> rotated_cube[8];

    QTime frame_rate_timer;
    int frames;
};

}

#endif // IMUFrame_H
