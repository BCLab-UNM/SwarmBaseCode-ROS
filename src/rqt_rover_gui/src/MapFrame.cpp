#ifndef rqt_rover_gui_MapFrame
#define rqt_rover_gui_MapFrame

#include <iostream>
#include <cmath>
#include <limits>
#include <QMainWindow>
#include <QGridLayout>
#include <QLabel>

#include "MapFrame.h"

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
    popout_mapframe = NULL;
    popout_window = NULL;


}

// This can't go in the constructor or there will be an infinite regression.
// Instead call from the main UI in it's startup routine.
void MapFrame::createPopoutWindow()
{
    popout_window = new QMainWindow();
    popout_mapframe = new MapFrame(popout_window, 0);

    QGridLayout* layout = new QGridLayout();
    layout->addWidget(popout_mapframe);

    QWidget* central_widget = new QWidget();
    central_widget->setLayout(layout);

    popout_window->setGeometry(QRect(10, 10, 500, 500));
    popout_window->setStyleSheet("background-color: rgb(0, 0, 0); border-color: rgb(255, 255, 255);");
    popout_window->setCentralWidget(central_widget);
}

void MapFrame::addToGPSRoverPath(string rover, float x, float y)
{     
  if(popout_mapframe) popout_mapframe->addToGPSRoverPath( rover, x, y );

  // Negate the y direction to orient the map so up is north.
  y = -y;

    if (x > max_gps_seen_x[rover]) max_gps_seen_x[rover] = x;
    if (y > max_gps_seen_y[rover]) max_gps_seen_y[rover] = y;
    if (x < min_gps_seen_x[rover]) min_gps_seen_x[rover] = x;
    if (y < min_gps_seen_y[rover]) min_gps_seen_y[rover] = y;

    update_mutex.lock();
    gps_rover_path[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}

void MapFrame::addToEncoderRoverPath(string rover, float x, float y)
{
  if(popout_mapframe) popout_mapframe->addToEncoderRoverPath( rover, x, y );

  // Negate the y direction to orient the map so up is north.
  y = -y;

    if (x > max_encoder_seen_x[rover]) max_encoder_seen_x[rover] = x;
    if (y > max_encoder_seen_y[rover]) max_encoder_seen_y[rover] = y;
    if (x < min_encoder_seen_x[rover]) min_encoder_seen_x[rover] = x;
    if (y < min_encoder_seen_y[rover]) min_encoder_seen_y[rover] = y;

    update_mutex.lock();
    encoder_rover_path[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();  
}


void MapFrame::addToEKFRoverPath(string rover, float x, float y)
{
  if(popout_mapframe) popout_mapframe->addToEKFRoverPath( rover, x, y );

  // Negate the y direction to orient the map so up is north.
  y = -y;

    if (x > max_ekf_seen_x[rover]) max_ekf_seen_x[rover] = x;
    if (y > max_ekf_seen_y[rover]) max_ekf_seen_y[rover] = y;
    if (x < min_ekf_seen_x[rover]) min_ekf_seen_x[rover] = x;
    if (y < min_ekf_seen_y[rover]) min_ekf_seen_y[rover] = y;

    update_mutex.lock();
    ekf_rover_path[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}

void MapFrame::clearMap()
{
    update_mutex.lock();
    for (auto rover : display_list)
    {
        clearMap(rover);
    }

    display_list.clear();
    update_mutex.unlock();

    if(popout_mapframe) popout_mapframe->clearMap();
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
    collection_points[rover].push_back(pair<float,float>(x,y));
    update_mutex.unlock();
    emit delayedUpdate();
}


void MapFrame::paintEvent(QPaintEvent* event)
{

    // Begin drawing the map
    QPainter painter(this);
    painter.setPen(Qt::white);
    QFont font = painter.font();
    qreal font_size = font.pointSizeF();
    QFontMetrics fm(font);

    // Track the frames per second for development purposes
    QString frames_per_second;
    frames_per_second = QString::number(frames / (frame_rate_timer.elapsed() / 1000.0), 'f', 0) + " FPS";


    painter.drawText(this->width()-fm.width(frames_per_second), fm.height(), frames_per_second);

    frames++;

    if (!(frames % 100)) // time how long it takes to dispay 100 frames
    {
        frame_rate_timer.start();
        frames = 0;
    }

    // end frames per second

    // Check if any rovers have been selected for display
    if ( display_list.empty() )
    {
        painter.drawText(QPoint(50,50), "No rover maps selected for display");
        return;
    }

    // Colorblind friendly colors
    QColor green(17, 192, 131);
    QColor red(255, 65, 30);

    float max_seen_x = -std::numeric_limits<float>::max(); // std::numeric_limits<float>::max() is the max possible floating point value
    float max_seen_y = -std::numeric_limits<float>::max();

    float min_seen_x = std::numeric_limits<float>::max();
    float min_seen_y = std::numeric_limits<float>::max();

    float max_seen_width = -std::numeric_limits<float>::max();
    float max_seen_height = -std::numeric_limits<float>::max();

    int no_data_offset = 0; // So the "no data" message is not overlayed if there are multiple rovers with no data.

    // Repeat the display code for each rover selected by the user - Using C++11 range syntax
    for(auto rover_to_display : display_list)
    {

    // Set the max and min seen values depending on which data the user has selected to view
    // Check each of the display data options and choose the most extreme value from those selected by the user

        // Always include the ekf data because that is what we are using to position the current position marker for the rover
    if (display_ekf_data)
    {
        if (min_seen_x > min_ekf_seen_x[rover_to_display]) min_seen_x = min_ekf_seen_x[rover_to_display];
        if (min_seen_y > min_ekf_seen_y[rover_to_display]) min_seen_y = min_ekf_seen_y[rover_to_display];
        if (max_seen_x < max_ekf_seen_x[rover_to_display]) max_seen_x = max_ekf_seen_x[rover_to_display];
        if (max_seen_y < max_ekf_seen_y[rover_to_display]) max_seen_y = max_ekf_seen_y[rover_to_display];
    }

    if (display_gps_data)
    {
        if (min_seen_x > min_gps_seen_x[rover_to_display]) min_seen_x = min_gps_seen_x[rover_to_display];
        if (min_seen_y > min_gps_seen_y[rover_to_display]) min_seen_y = min_gps_seen_y[rover_to_display];
        if (max_seen_x < max_gps_seen_x[rover_to_display]) max_seen_x = max_gps_seen_x[rover_to_display];
        if (max_seen_y < max_gps_seen_y[rover_to_display]) max_seen_y = max_gps_seen_y[rover_to_display];
    }

    if (display_encoder_data)
    {
        if (min_seen_x > min_encoder_seen_x[rover_to_display]) min_seen_x = min_encoder_seen_x[rover_to_display];
        if (min_seen_y > min_encoder_seen_y[rover_to_display]) min_seen_y = min_encoder_seen_y[rover_to_display];
        if (max_seen_x < max_encoder_seen_x[rover_to_display]) max_seen_x = max_encoder_seen_x[rover_to_display];
        if (max_seen_y < max_encoder_seen_y[rover_to_display]) max_seen_y = max_encoder_seen_y[rover_to_display];
    }

    // Normalize the displayed coordinates to the largest coordinates seen since we don't know the coordinate system.
    max_seen_width = max_seen_x-min_seen_x;
    max_seen_height = max_seen_y-min_seen_y;


    if (ekf_rover_path[rover_to_display].empty() && encoder_rover_path[rover_to_display].empty() && gps_rover_path[rover_to_display].empty() && target_locations[rover_to_display].empty() && collection_points[rover_to_display].empty())
    {
        painter.drawText(QPoint(50,50+no_data_offset), QString::fromStdString(rover_to_display) + ": No data.");
        no_data_offset += 10;
    }

    // Check extended kalman filter has any values in it
    else if (ekf_rover_path[rover_to_display].empty())
       {
            painter.drawText(QPoint(50,50+no_data_offset), "Map Frame: No EKF data received.");
            no_data_offset += 10;
        }
   }

    int map_origin_x = fm.width(QString::number(-max_seen_height, 'f', 1)+"m");
    int map_origin_y = 2*fm.height();

    int map_width = this->width()-map_origin_x;
    int map_height = this->height()-map_origin_y;

    int map_center_x = map_origin_x+((map_width-map_origin_x)/2);
    int map_center_y = map_origin_y+((map_height-map_origin_y)/2);

    // The map axes do not need to be redrawn for each rover so this code is sandwiched between the two rover display list loops


    // Draw the scale bars
    //painter.setPen(Qt::gray);
    //painter.drawLine(QPoint(map_center_x, map_origin_y), QPoint(map_center_x, map_height));
    //painter.drawLine(QPoint(map_origin_x, map_center_y), QPoint(map_width, map_center_y));
    //painter.setPen(Qt::white);

    // Cross hairs at map display center
    QPoint axes_origin(map_origin_x,map_origin_y);
    QPoint x_axis(map_width,map_origin_y);
    QPoint y_axis(map_origin_x,map_height);
    painter.drawLine(axes_origin, x_axis);
    painter.drawLine(axes_origin, y_axis);
    painter.drawLine(QPoint(map_width, map_origin_y), QPoint(map_width, map_height));
    painter.drawLine(QPoint(map_origin_x, map_height), QPoint(map_width, map_height));

    // Draw north arrow
    QPoint northArrow_point(map_center_x, 0);
    QPoint northArrow_left(map_center_x - 5, 5);
    QPoint northArrow_right(map_center_x + 5, 5);
    QRect northArrow_textBox(northArrow_left.x(), northArrow_left.y(), 10, 15);
    painter.drawLine(northArrow_left, northArrow_right);
    painter.drawLine(northArrow_left, northArrow_point);
    painter.drawLine(northArrow_right, northArrow_point);
    painter.drawText(northArrow_textBox, QString("N"));

    // Draw rover origin crosshairs
    // painter.setPen(green);


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

    // Repeat the display code for each rover selected by the user - Using C++11 range syntax
    for(auto rover_to_display : display_list)
    {
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

        // Move to the starting point of the path without drawing a line
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

        // Move to the starting point of the path without drawing a line
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

    // Draw a yellow circle at the current EKF estimated rover location
    painter.setPen(Qt::yellow);
    pair<float,float> current_coordinate = ekf_rover_path[rover_to_display].back();
    QPoint point;
    float x = map_origin_x+((current_coordinate.first-min_seen_x)/max_seen_width)*(map_width-map_origin_x);
    float y = map_origin_y+((current_coordinate.second-min_seen_y)/max_seen_height)*(map_height-map_origin_y);
    float radius = 2.5;
    //painter.drawArc(x-radius,y-radius,2*radius,2*radius,0,16*360);
    painter.drawEllipse(QPointF(x,y), radius, radius);

    // Display rover name in half the default font size
    font.setPointSizeF( font_size / 2);

    // set the modified font to the painter
    painter.setFont(font);
    painter.drawText(QPoint(x,y), QString::fromStdString(rover_to_display));

    // return the painter font size to normal
    font.setPointSizeF( font_size );

    update_mutex.unlock();

    painter.setPen(Qt::white);
  } // End rover display list set iteration


    painter.setPen(Qt::blue);
    painter.drawText(QPoint(0,15), "min_seen_x: " + QString::number(min_seen_x));
    painter.drawText(QPoint(0,30), "min_seen_y: " + QString::number(min_seen_y));
    painter.drawText(QPoint(0,45), "max_width_seen: " + QString::number(max_seen_width));
    painter.drawText(QPoint(0,60), "max_height_seen: " + QString::number(max_seen_height));

    painter.drawText(QPoint(0,75), "map_origin_x: " + QString::number(map_origin_x));
    painter.drawText(QPoint(0,90), "map_origin_y: " + QString::number(map_origin_y));
    painter.drawText(QPoint(0,105), "map_width: " + QString::number(map_width));
    painter.drawText(QPoint(0,120), "map_height: " + QString::number(map_height));
    painter.setPen(Qt::white);
}


void MapFrame::setDisplayEncoderData(bool display)
{
    display_encoder_data = display;

    if(popout_mapframe) popout_mapframe->setDisplayEncoderData(display);
}

void MapFrame::setDisplayGPSData(bool display)
{
    display_gps_data = display;

    if(popout_mapframe) popout_mapframe->setDisplayGPSData(display);
}

void MapFrame::setDisplayEKFData(bool display)
{
    display_ekf_data = display;

    if(popout_mapframe) popout_mapframe->setDisplayEKFData(display);
}

void MapFrame::setWhetherToDisplay(string rover, bool yes)
{
    if (yes)
    {
        display_list.insert(rover);
    }
    else
    {
        display_list.erase(rover);
    }

    if(popout_mapframe) popout_mapframe->setWhetherToDisplay(rover, yes);
}

void MapFrame::mousePressEvent(QMouseEvent *event)
{
    emit sendInfoLogMessage("MapFrame: mouse press.");

    if (popout_window) popout_window->show();
}

void MapFrame::mouseMoveEvent(QMouseEvent *event)
{
    emit sendInfoLogMessage("MapFrame: mouse move.");
}

void MapFrame::wheelEvent(QWheelEvent *e)
{
    emit sendInfoLogMessage("MapFrame: mouse wheel.");
}

MapFrame::~MapFrame()
{
    // Safely erase map data - locks to make sure a frame isnt being drawn
    clearMap();
    if (popout_window) delete popout_window;
}

}
#endif
