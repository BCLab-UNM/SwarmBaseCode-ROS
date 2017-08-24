/*!
 * \brief  This frame draws the images recieved from the ROS camera subscriber.
 * \author Matthew Fricke
 * \date   November 11th 2015
 * \todo   Code works properly.
 * \class  CameraFrame
 */

#ifndef CAMERAFRAME_H
#define CAMERAFRAME_H

#include <cmath>
#include <utility>
#include <vector>
#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <QPointF>

namespace rqt_rover_gui
{
  class CameraFrame : public QFrame {
    Q_OBJECT

    public:
      CameraFrame(QWidget *parent, Qt::WindowFlags = 0);
      void setImage(const QImage& image);
      // four corners of tag
      void addTarget(std::pair<double,double> c1, std::pair<double,double> c2,
                     std::pair<double,double> c3, std::pair<double,double> c4,
                     std::pair<double,double> center);

    signals:
      void delayedUpdate();

    private slots:
      // currently, there are no class-defined slots in use

    protected:
      void paintEvent(QPaintEvent *event);

    private:
      QImage image;
      mutable QMutex image_update_mutex;

      QTime frame_rate_timer;
      int frames;

      std::vector<std::pair<double, double>> target_corners_1;
      std::vector<std::pair<double, double>> target_corners_2;
      std::vector<std::pair<double, double>> target_corners_3;
      std::vector<std::pair<double, double>> target_corners_4;
      std::vector<std::pair<double, double>> target_centers;
  };
}

#endif // CAMERAFRAME_H
