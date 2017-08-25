/*!
 * \brief  This frame is intended to show information about the quality of
 *         the GPS sensor data, for example, the number of satellites current
 *         sending data.
 * \author Matthew Fricke
 * \date   November 11th 2015
 * \todo   This frame is not currently being used and is currently just a place
 *         holder class.
 * \class  GPSFrame
 */

#ifndef GPSFRAME_H
#define GPSFRAME_H

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <utility> // for STL pair

using namespace std;

namespace rqt_rover_gui
{
  class GPSFrame : public QFrame
  {
    Q_OBJECT

    public:
      GPSFrame(QWidget *parent, Qt::WindowFlags = 0);

    signals:
      void delayedUpdate();
      
    public slots:
      // currently, no class-defined slots are being used

    protected:
      void paintEvent(QPaintEvent *event);

    private:
      QTime frame_rate_timer;
      int frames;
  };
}

#endif // GPSFrame_H
