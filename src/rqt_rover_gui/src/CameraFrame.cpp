#ifndef rqt_rover_gui_CameraFrame
#define rqt_rover_gui_CameraFrame

#include <CameraFrame.h>

namespace rqt_rover_gui
{

CameraFrame::CameraFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);
}

void CameraFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);


    image_update_mutex.lock();
    if (!(image.isNull()))
    {
        //painter.drawText(QPoint(50,50), "Image Received From Camera");

        painter.drawImage(contentsRect(), image);
    }
    else
    {

        painter.drawText(QPoint(50,50), "No Image Received From Camera");
    }
    image_update_mutex.unlock();
}

void CameraFrame::setImage(const QImage& img)
{
    image_update_mutex.lock();
    image = img.copy();
    image_update_mutex.unlock();
    emit delayedUpdate();
}

}
#endif
