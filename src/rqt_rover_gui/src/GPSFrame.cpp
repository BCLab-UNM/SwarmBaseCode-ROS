#ifndef rqt_rover_gui_GPSFrame
#define rqt_rover_gui_GPSFrame

#include <iostream>
#include <cmath>
#include <GPSFrame.h>

namespace rqt_rover_gui
{

GPSFrame::GPSFrame(QWidget *parent, Qt::WindowFlags flags) : QFrame(parent) {
  connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()),
          Qt::QueuedConnection);
  frames = 0;
}

void GPSFrame::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setPen(Qt::white);

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

  painter.drawText(QPoint(50,50), "GPS Frame");
  painter.setPen(Qt::white);
}

} /* END: namespace rqt_rover_gui */

#endif /* rqt_rover_gui_GPSFrame */
