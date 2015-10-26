#ifndef rtq_rover_gui_IMUWidget_H
#define rtq_rover_gui_IMUWidget_H

#include <QtOpenGL>
#include <QGLWidget>
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <tuple>
#include <utility> // For STL pair and make_tuple

using namespace std;

namespace rqt_rover_gui
{

class IMUWidget : public QGLWidget
{
    Q_OBJECT
public:
    IMUWidget(QWidget *parent, Qt::WFlags = 0);
    void setLinearAcceleration(float x, float y, float z);
    void setAngularVelocity(float x, float y, float z);
    void setOrientation(float w, float x, float y, float z);

    // OpenGL Functions
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    void saveGLState();
    void restoreGLState();
    void paintGL();
    void initializeGL();
    void resizeGL(int width, int height);

signals:

    void delayedUpdate();

public slots:


protected:

private:

tuple<float, float, float> linear_acceleration; // ROS Geometry Messages Vector3: <x, y, z>
tuple<float, float, float> angular_velocity; // ROS Geometry Messages Vector3: <x, y, z>
tuple<float, float, float, float> orientation; // ROS Geometry Messages Quaternion: <w, x, y, z>

};

}

#endif // IMUWidget_H
