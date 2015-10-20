#ifndef rqt_rover_gui_MapFrame
#define rqt_rover_gui_MapFrame

#include <iostream>
#include <cmath>

#include <MapFrame.h>

namespace rqt_rover_gui
{

MapFrame::MapFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);
    // Scale coordinates
    frame_width = this->width();
    frame_height = this->height();

    max_seen_x = 1;
    max_seen_y = 1;
}

void MapFrame::addToRoverPath(float x, float y)
{
    if (x > max_seen_x) max_seen_x = x;
    if (y > max_seen_y) max_seen_y = y;
    if (x < min_seen_x) min_seen_x = x;
    if (y < min_seen_y) min_seen_y = y;

    // Normalize the displayed coordinates to the largest coordinates seen since we don't know the coordinate system.
    max_seen_width = std::fabs(min_seen_x)+std::fabs(max_seen_x);
    max_seen_height = std::fabs(min_seen_y)+std::fabs(max_seen_y);

    update_mutex.lock();
    rover_path.push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}

void MapFrame::clearMap()
{
    rover_path.clear();
    target_locations.clear();
    collection_points.clear();
}

void MapFrame::addTargetLocation(float x, float y)
{
    update_mutex.lock();
    target_locations.push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}


void MapFrame::addCollectionPoint(float x, float y)
{
    update_mutex.lock();
    collection_points.push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}


void MapFrame::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setPen(Qt::white);

    frame_width = this->width();
    frame_height = this->height();

    update_mutex.lock();
    // scale coordinates

    std::vector<QPoint> scaled_target_locations;
    for(std::vector< pair<float,float> >::iterator it = target_locations.begin(); it != target_locations.end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        point.setX(coordinate.first*frame_width);
        point.setY(coordinate.second*frame_height);
        scaled_target_locations.push_back(point);
    }

    std::vector<QPoint> scaled_collection_points;
    for(std::vector< pair<float,float> >::iterator it = collection_points.begin(); it != collection_points.end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        point.setX(coordinate.first*frame_width);
        point.setY(coordinate.second*frame_height);
        scaled_collection_points.push_back(point);
    }

    QPainterPath scaled_rover_path;
    for(std::vector< pair<float,float> >::iterator it = rover_path.begin(); it != rover_path.end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        float x = ((coordinate.first-min_seen_x)/max_seen_width)*frame_width;
        float y = ((coordinate.second-min_seen_y)/max_seen_height)*frame_height;

        scaled_rover_path.lineTo(x, y);
    }

    painter.drawPath(scaled_rover_path);
    painter.setPen(Qt::red);
    QPoint* point_array = &scaled_collection_points[0];
    painter.drawPoints(point_array, scaled_collection_points.size());
    painter.setPen(Qt::green);
    point_array = &scaled_target_locations[0];
    painter.drawPoints(point_array, scaled_target_locations.size());

    update_mutex.unlock();

    painter.setPen(Qt::white);
}

}
#endif
