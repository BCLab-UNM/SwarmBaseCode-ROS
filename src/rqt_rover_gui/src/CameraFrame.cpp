#include <CameraFrame.h>

namespace rqt_rover_gui
{

CameraFrame::CameraFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);

        frames = 0;
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

}

void CameraFrame::setImage(const QImage& img)
{
    image_update_mutex.lock();
    image = img.copy();
    image_update_mutex.unlock();
    emit delayedUpdate();
}

}
