#ifndef rqt_rover_gui_MapFrame
#define rqt_rover_gui_MapFrame

#include <iostream>
#include <cmath>
#include <limits>

#include <MapFrame.h>


namespace rqt_rover_gui
{

MapFrame::MapFrame(QWidget *parent, Qt::WFlags flags) : QFrame(parent)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);
    // Scale coordinates
    frame_width = this->width();
    frame_height = this->height();

//    max_gps_seen_x[rover] = -std::numeric_limits<float>::max();
//    max_gps_seen_y[rover] = -std::numeric_limits<float>::max();
//    min_gps_seen_x[rover] = std::numeric_limits<float>::max();
//    min_gps_seen_y[rover] = std::numeric_limits<float>::max();

//    max_ekf_seen_x[rover] = -std::numeric_limits<float>::max();
//    max_ekf_seen_y[rover] = -std::numeric_limits<float>::max();
//    min_ekf_seen_x[rover] = std::numeric_limits<float>::max();
//    min_ekf_seen_y[rover] = std::numeric_limits<float>::max();

//    max_encoder_seen_x[rover] = -std::numeric_limits<float>::max();
//    max_encoder_seen_y[rover] = -std::numeric_limits<float>::max();
//    min_encoder_seen_x[rover] = std::numeric_limits<float>::max();
//    min_encoder_seen_y[rover] = std::numeric_limits<float>::max();

//    max_gps_seen_width[rover] = 0;
//    max_gps_seen_height[rover] = 0;

//    max_ekf_seen_width[rover] = 0;
//    max_ekf_seen_height[rover] = 0;

//    max_encoder_seen_width[rover] = 0;
//    max_encoder_seen_height[rover] = 0;

    display_ekf_data = false;
    display_gps_data = false;
    display_encoder_data = false;

    frames = 0;

}

void MapFrame::setRoverMapToDisplay(string rover)
{
    rover_to_display = rover;
}

void MapFrame::addToGPSRoverPath(string rover, float x, float y)
{ 
  // Negate the y direction to orient the map so up is north.
  y = -y;

    if (x > max_gps_seen_x[rover]) max_gps_seen_x[rover] = x;
    if (y > max_gps_seen_y[rover]) max_gps_seen_y[rover] = y;
    if (x < min_gps_seen_x[rover]) min_gps_seen_x[rover] = x;
    if (y < min_gps_seen_y[rover]) min_gps_seen_y[rover] = y;

    // Normalize the displayed coordinates to the largest coordinates seen since we don't know the coordinate system.
    max_gps_seen_width[rover] = max_gps_seen_x[rover]-min_gps_seen_x[rover];
    max_gps_seen_height[rover] = max_gps_seen_y[rover]-min_gps_seen_y[rover];

    update_mutex.lock();
    gps_rover_path[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}

void MapFrame::addToEncoderRoverPath(string rover, float x, float y)
{
  // Negate the y direction to orient the map so up is north.
  y = -y;

    if (x > max_encoder_seen_x[rover]) max_encoder_seen_x[rover] = x;
    if (y > max_encoder_seen_y[rover]) max_encoder_seen_y[rover] = y;
    if (x < min_encoder_seen_x[rover]) min_encoder_seen_x[rover] = x;
    if (y < min_encoder_seen_y[rover]) min_encoder_seen_y[rover] = y;

    // Normalize the displayed coordinates to the largest coordinates seen since we don't know the coordinate system.
    max_encoder_seen_width[rover] = max_encoder_seen_x[rover]-min_encoder_seen_x[rover];
    max_encoder_seen_height[rover] = max_encoder_seen_y[rover]-min_encoder_seen_y[rover];

    update_mutex.lock();
    encoder_rover_path[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}


void MapFrame::addToEKFRoverPath(string rover, float x, float y)
{
  // Negate the y direction to orient the map so up is north.
  y = -y;

    if (x > max_ekf_seen_x[rover]) max_ekf_seen_x[rover] = x;
    if (y > max_ekf_seen_y[rover]) max_ekf_seen_y[rover] = y;
    if (x < min_ekf_seen_x[rover]) min_ekf_seen_x[rover] = x;
    if (y < min_ekf_seen_y[rover]) min_ekf_seen_y[rover] = y;

    // Normalize the displayed coordinates to the largest coordinates seen since we don't know the coordinate system.
    max_ekf_seen_width[rover] = max_ekf_seen_x[rover]-min_ekf_seen_x[rover];
    max_ekf_seen_height[rover] = max_ekf_seen_y[rover]-min_ekf_seen_y[rover];

    update_mutex.lock();
    ekf_rover_path[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}

void MapFrame::clearMap(string rover)
{
    ekf_rover_path[rover].clear();
    encoder_rover_path[rover].clear();
    gps_rover_path[rover].clear();
    target_locations[rover].clear();
    collection_points[rover].clear();

    ekf_rover_path.erase(rover);
    encoder_rover_path.erase(rover);
    gps_rover_path.erase(rover);
    target_locations.erase(rover);
    collection_points.erase(rover);
}

void MapFrame::addTargetLocation(string rover, float x, float y)
{
  //The QT drawing coordinate system is reversed from the robot coordinate system in the y direction
    y = -y;

    update_mutex.lock();
    target_locations[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}


void MapFrame::addCollectionPoint(string rover, float x, float y)
{
    // The QT drawing coordinate system is reversed from the robot coordinate system in the y direction
    y = -y;

    update_mutex.lock();
    collection_points[rover_to_display].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}


void MapFrame::paintEvent(QPaintEvent* event)
{
    // Colorblind friendly colors
    QColor green(17, 192, 131);
    QColor red(255, 65, 30);

    float max_seen_height = -std::numeric_limits<float>::max(); // std::numeric_limits<float>::max() is the max possible floating point value
    float max_seen_width = -std::numeric_limits<float>::max();

    float min_seen_x = std::numeric_limits<float>::max();
    float min_seen_y = std::numeric_limits<float>::max();

    // Set the max and min seen values depending on which data the user has selected to view
    // Check each of the display data options and choose the most extreme value from those selected by the user
    if (display_ekf_data)
    {
        min_seen_x = min_ekf_seen_x[rover_to_display];
        min_seen_y = min_ekf_seen_y[rover_to_display];
        max_seen_height = max_ekf_seen_height[rover_to_display];
        max_seen_width = max_ekf_seen_height[rover_to_display];
    }

    if (display_gps_data)
    {
        if (min_seen_x > min_gps_seen_x[rover_to_display]) min_seen_x = min_gps_seen_x[rover_to_display];
        if (min_seen_y > min_gps_seen_y[rover_to_display]) min_seen_y = min_gps_seen_y[rover_to_display];
        if (max_seen_height < max_gps_seen_height[rover_to_display]) max_seen_height = max_gps_seen_height[rover_to_display];
        if (max_seen_width < max_gps_seen_width[rover_to_display]) max_seen_width = max_gps_seen_width[rover_to_display];
    }

    if (display_encoder_data)
    {
        if (min_seen_x > min_encoder_seen_x[rover_to_display]) min_seen_x = min_encoder_seen_x[rover_to_display];
        if (min_seen_y > min_encoder_seen_y[rover_to_display]) min_seen_y = min_encoder_seen_y[rover_to_display];
        if (max_seen_height < max_encoder_seen_height[rover_to_display]) max_seen_height = max_encoder_seen_height[rover_to_display];
        if (max_seen_width < max_encoder_seen_width[rover_to_display]) max_seen_width = max_encoder_seen_width[rover_to_display];
    }

    // Begin drawing the map
    QPainter painter(this);
    painter.setPen(Qt::white);

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

    if (ekf_rover_path[rover_to_display].empty() && encoder_rover_path[rover_to_display].empty() && gps_rover_path[rover_to_display].empty() && target_locations[rover_to_display].empty() && collection_points[rover_to_display].empty())
    {
        painter.drawText(QPoint(50,50), "Map Frame: Nothing to display.");
        return;
    }

    int map_origin_x = fm.width(QString::number(-max_seen_height, 'f', 1)+"m");
    int map_origin_y = 2*fm.height();

    int map_width = this->width()-map_origin_x;
    int map_height = this->height()-map_origin_y;

    int map_center_x = map_origin_x+((map_width-map_origin_x)/2);
    int map_center_y = map_origin_y+((map_height-map_origin_y)/2);

    // Draw the scale bars
    //painter.setPen(Qt::gray);
    //painter.drawLine(QPoint(map_center_x, map_origin_y), QPoint(map_center_x, map_height));
    //painter.drawLine(QPoint(map_origin_x, map_center_y), QPoint(map_width, map_center_y));
    //painter.setPen(Qt::white);

    // cross hairs at map display center
    QPoint axes_origin(map_origin_x,map_origin_y);
    QPoint x_axis(map_width,map_origin_y);
    QPoint y_axis(map_origin_x,map_height);
    painter.drawLine(axes_origin, x_axis);
    painter.drawLine(axes_origin, y_axis);
    painter.drawLine(QPoint(map_width, map_origin_y), QPoint(map_width, map_height));
    painter.drawLine(QPoint(map_origin_x, map_height), QPoint(map_width, map_height));

    // Draw rover origin crosshairs
    // painter.setPen(green);

    // Check encoder has any values in it
    if (ekf_rover_path[rover_to_display].empty())
          {
            painter.drawText(QPoint(50,50), "Map Frame: No encoder data received.");
           return;
        }

    float initial_x = 0.0; //ekf_rover_path[rover_to_display].begin()->first;
    float initial_y = 0.001; //ekf_rover_path[rover_to_display].begin()->second;
    float rover_origin_x = map_origin_x+((initial_x-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
    float rover_origin_y = map_origin_y+((initial_y-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
    painter.setPen(Qt::gray);
    painter.drawLine(QPoint(rover_origin_x, map_origin_y), QPoint(rover_origin_x, map_height));
    painter.drawLine(QPoint(map_origin_x, rover_origin_y), QPoint(map_width, rover_origin_y));
    painter.setPen(Qt::white);

    int n_ticks = 6;
    float tick_length = 5;
    QPoint x_axis_ticks[n_ticks];
    QPoint y_axis_ticks[n_ticks];

    for (int i = 0; i < n_ticks-1; i++)
    {
        x_axis_ticks[i].setX(axes_origin.x()+(i+1)*map_width/n_ticks);
        x_axis_ticks[i].setY(axes_origin.y());

        y_axis_ticks[i].setX(axes_origin.x());
        y_axis_ticks[i].setY(axes_origin.y()+(i+1)*map_height/n_ticks);
    }

    for (int i = 0; i < n_ticks-1; i++)
    {
        painter.drawLine(x_axis_ticks[i], QPoint(x_axis_ticks[i].x(), x_axis_ticks[i].y()+tick_length));
        painter.drawLine(y_axis_ticks[i], QPoint(y_axis_ticks[i].x()+tick_length, y_axis_ticks[i].y()));
    }

    for (int i = 0; i < n_ticks-1; i++)
    {
        float fraction_of_map_to_rover_x = (rover_origin_x-map_origin_x)/map_width;
        float fraction_of_map_to_rover_y = (rover_origin_y-map_origin_y)/map_height;
        float x_label_f = (i+1)*max_seen_width/n_ticks-fraction_of_map_to_rover_x*max_seen_width;
        float y_label_f = (i+1)*max_seen_height/n_ticks-fraction_of_map_to_rover_y*max_seen_height;

        QString x_label = QString::number(x_label_f, 'f', 1) + "m";
        QString y_label = QString::number(-y_label_f, 'f', 1) + "m";

        int x_labels_offset_x = -(fm.width(x_label))/2;
        int x_labels_offset_y = 0;

        int y_labels_offset_x = -(fm.width(y_label));
        int y_labels_offset_y = fm.height()/3;

        painter.drawText(x_axis_ticks[i].x()+x_labels_offset_x, axes_origin.y()+x_labels_offset_y, x_label);
        painter.drawText(axes_origin.x()+y_labels_offset_x, y_axis_ticks[i].y()+y_labels_offset_y, y_label);

    }

    // End draw scale bars

    update_mutex.lock();
    // scale coordinates

    std::vector<QPoint> scaled_target_locations;
    for(std::vector< pair<float,float> >::iterator it = target_locations[rover_to_display].begin(); it != target_locations[rover_to_display].end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        point.setX(map_origin_x+coordinate.first*map_width);
        point.setY(map_origin_y+coordinate.second*map_height);
        scaled_target_locations.push_back(point);
    }

    std::vector<QPoint> scaled_collection_points;
    for(std::vector< pair<float,float> >::iterator it = collection_points[rover_to_display].begin(); it != collection_points[rover_to_display].end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        point.setX(map_origin_x+coordinate.first*map_width);
        point.setY(map_origin_y+coordinate.second*map_height);
        scaled_collection_points.push_back(point);
    }

    std::vector<QPoint> scaled_gps_rover_points;
    for(std::vector< pair<float,float> >::iterator it = gps_rover_path[rover_to_display].begin(); it != gps_rover_path[rover_to_display].end(); ++it) {
        pair<float,float> coordinate  = *it;

        float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
        float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
        scaled_gps_rover_points.push_back( QPoint(x,y) );
    }


    QPainterPath scaled_ekf_rover_path;
    for(std::vector< pair<float,float> >::iterator it = ekf_rover_path[rover_to_display].begin(); it != ekf_rover_path[rover_to_display].end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
        float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);

        if (it == ekf_rover_path[rover_to_display].begin()) scaled_ekf_rover_path.moveTo(x, y);
        scaled_ekf_rover_path.lineTo(x, y);
    }

    QPainterPath scaled_gps_rover_path;
    for(std::vector< pair<float,float> >::iterator it = gps_rover_path[rover_to_display].begin(); it != gps_rover_path[rover_to_display].end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
        float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);

        scaled_gps_rover_path.lineTo(x, y);
    }

    QPainterPath scaled_encoder_rover_path;
    for(std::vector< pair<float,float> >::iterator it = encoder_rover_path[rover_to_display].begin(); it != encoder_rover_path[rover_to_display].end(); ++it) {
        pair<float,float> coordinate  = *it;
        QPoint point;
        float x = map_origin_x+((coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
        float y = map_origin_y+((coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);

       if (it == encoder_rover_path[rover_to_display].begin()) scaled_encoder_rover_path.moveTo(x, y);

        scaled_encoder_rover_path.lineTo(x, y);
    }


    painter.setPen(red);
    if (display_gps_data) painter.drawPoints(&scaled_gps_rover_points[0], scaled_gps_rover_points.size());
   // if (display_gps_data) painter.drawPath(scaled_gps_rover_path);

    painter.setPen(Qt::white);
    if (display_ekf_data) painter.drawPath(scaled_ekf_rover_path);
    painter.setPen(green);
    if (display_encoder_data) painter.drawPath(scaled_encoder_rover_path);


    painter.setPen(red);
    QPoint* point_array = &scaled_collection_points[0];
    painter.drawPoints(point_array, scaled_collection_points.size());
    painter.setPen(green);
    point_array = &scaled_target_locations[0];
    painter.drawPoints(point_array, scaled_target_locations.size());

    update_mutex.unlock();

    painter.setPen(Qt::white);
}


void MapFrame::setDisplayEncoderData(bool display)
{
    display_encoder_data = display;
}

void MapFrame::setDisplayGPSData(bool display)
{
    display_gps_data = display;
}

void MapFrame::setDisplayEKFData(bool display)
{
    display_ekf_data = display;
}


}
#endif
