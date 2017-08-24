/*!
 * \brief  This USFrame (ultrasound frame) class visualizes the position output
 *         from the three ultrasound sensors. The ultrasound output is shown as
 *         three white rays, one for each ultrasound. The length of the rays
 *         indicates the distance to any objects in front of the ultrasound. The
 *         distance in meters is displayed in text below these rays. The maximum
 *         distance reported by the ultrasounds is 3 meters.
 *
 * \author Matthew Fricke
 * \date   November 11th 2015
 * \todo   Code works properly.
 * \class  USFrame
 */

#ifndef USFRAME_H
#define USFRAME_H

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
#include <utility> // for STL pair

using namespace std;

namespace rqt_rover_gui {
  class USFrame : public QFrame {
    Q_OBJECT

    public:
      USFrame(QWidget *parent, Qt::WindowFlags = 0);
      void setCenterRange(float r, float min, float max);
      void setLeftRange(float r, float min, float max);
      void setRightRange(float r, float min, float max);

    signals:
      void delayedUpdate();

    public slots:
      // currently, no class-defined slots are being used

    protected:
      void paintEvent(QPaintEvent *event);

    private:
      float center_range;
      float left_range;
      float right_range;

      float center_max_range;
      float center_min_range;

      float left_max_range;
      float left_min_range;

      float right_max_range;
      float right_min_range;

      QTime frame_rate_timer;
      int frames;
  };
} /* END: namespace rqt_rover_gui */

#endif // USFrame_H
