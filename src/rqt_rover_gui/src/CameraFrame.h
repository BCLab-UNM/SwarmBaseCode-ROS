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

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>

namespace rqt_rover_gui
{

class CameraFrame : public QFrame
{
    Q_OBJECT
public:
    CameraFrame(QWidget *parent, Qt::WFlags = 0);

    void setImage(const QImage& image);

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
};

}

#endif // CAMERAFRAME_H
