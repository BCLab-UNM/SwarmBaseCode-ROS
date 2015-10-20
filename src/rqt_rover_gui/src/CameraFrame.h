#ifndef rtq_rover_gui_CAMERAFRAME_H
#define rtq_rover_gui_CAMERAFRAME_H

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

};

}

#endif // CAMERAFRAME_H
