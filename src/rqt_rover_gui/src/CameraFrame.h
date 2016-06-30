/*!
 * \brief   This frame draws the images recieved from the ROS
 *          camera subscriber.
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo    Code works properly.
 * \class   CameraFrame
 */

#ifndef CAMERAFRAME_H
#define CAMERAFRAME_H

#include <cmath>
#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <QPointF>
#include <sstream>
#include <ros/ros.h>

namespace rqt_rover_gui
{

class CameraFrame : public QFrame
{
    Q_OBJECT
public:
    CameraFrame(QWidget *parent, Qt::WFlags = 0);

    void setImage(const QImage& image);
    // four corners of tag
    void addTarget(double c1[2], double c2[2], double c3[2], double c4[2]);
    bool greaterThan(double c1[2], double c2[2]);
    bool greaterThan(double c1, double c2);

signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    QImage image;
    mutable QMutex image_update_mutex;

    QTime frame_rate_timer;
    int frames;

    double target_c1[2];
    double target_c2[2];
    double target_c3[2];
    double target_c4[2];
};

}

#endif // CAMERAFRAME_H
