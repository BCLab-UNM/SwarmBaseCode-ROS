#include <CameraFrame.h>

namespace rqt_rover_gui {

CameraFrame::CameraFrame(QWidget *parent, Qt::WindowFlags flags) : QFrame(parent)
{
  connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()),
          Qt::QueuedConnection);

  frames = 0;
}

void CameraFrame::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setPen(Qt::white);

  image_update_mutex.lock();

  if (!(image.isNull())) {
    painter.drawImage(contentsRect(), image);

    if(target_corners_1.size() > 0) {
      QPen pen(Qt::red);
      pen.setWidth(2);
      painter.setPen(pen);

      for(int i = 0; i < target_corners_1.size(); i++)  {
        QPointF points[4] = {
          QPointF(target_corners_1[i].first, target_corners_1[i].second),
          QPointF(target_corners_2[i].first, target_corners_2[i].second),
          QPointF(target_corners_3[i].first, target_corners_3[i].second),
          QPointF(target_corners_4[i].first, target_corners_4[i].second),
        };

        painter.drawPolygon(points, 4);
      }

      painter.setPen(Qt::white);
    }

    target_corners_1.clear();
    target_corners_2.clear();
    target_corners_3.clear();
    target_corners_4.clear();
  } else {
    painter.drawText(QPoint(50,50), "No Image Received From Camera");
  }

  image_update_mutex.unlock();

  // Track the frames per second for development purposes
  float fps = (float)frames / ((float)frame_rate_timer.elapsed() / 1000.0);
  QString frames_per_second = QString::number(fps, 'f', 0) + " FPS";

  QFontMetrics fm(painter.font());
  painter.drawText(this->width()-fm.width(frames_per_second), fm.height(),
                   frames_per_second);

  frames++;

  // time how long it takes to dispay 100 frames
  if (!(frames % 100)) {
    frame_rate_timer.start();
    frames = 0;
  }

  // end frames per second
}

void CameraFrame::setImage(const QImage& img) {
    image_update_mutex.lock();
    image = img.copy();
    image_update_mutex.unlock();
    emit delayedUpdate();
}

void CameraFrame::addTarget(std::pair<double,double> c1,
                            std::pair<double,double> c2,
                            std::pair<double,double> c3,
                            std::pair<double,double> c4,
                            std::pair<double,double> center) {
    target_corners_1.push_back(c1);
    target_corners_2.push_back(c2);
    target_corners_3.push_back(c3);
    target_corners_4.push_back(c4);
    target_centers.push_back(center);
}

} /* END: namespace rqt_rover_gui */

