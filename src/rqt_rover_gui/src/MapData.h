#ifndef MAPDATA_H
#define MAPDATA_H

#include <vector>
#include <set>
#include <utility> // For STL std::pair
#include <map>
#include <string>
#include <QMutex>

#include "MapData.h"

// This class is the "model" for std::map frame in the model-view UI pattern,
// where std::mapFrame is the view.
// This allows the creation of multiple std::maps without duplicating large amounts of std::map data.
class MapData
{
public:
    MapData();

    void addToGPSRoverPath(std::string rover, float x, float y);
    void addToEncoderRoverPath(std::string rover, float x, float y);
    void addToEKFRoverPath(std::string rover, float x, float y);
    void addTargetLocation(std::string rover, float x, float y);
    void addCollectionPoint(std::string rover, float x, float y);

    void clear();
    void clear(std::string rover_name);
    void lock();
    void unlock();

    std::vector< std::pair<float,float> >* getEKFPath(std::string rover_name);
    std::vector< std::pair<float,float> >* getGPSPath(std::string rover_name);
    std::vector< std::pair<float,float> >* getEncoderPath(std::string rover_name);
    std::vector< std::pair<float,float> >* getTargetLocations(std::string rover_name);
    std::vector< std::pair<float,float> >* getCollectionPoints(std::string rover_name);

    // These functions provide a fast way to get the min and max coords
    float getMaxGPSX(std::string rover_name);
    float getMaxGPSY(std::string rover_name);
    float getMinGPSX(std::string rover_name);
    float getMinGPSY(std::string rover_name);

    float getMaxEKFX(std::string rover_name);
    float getMaxEKFY(std::string rover_name);
    float getMinEKFX(std::string rover_name);
    float getMinEKFY(std::string rover_name);

    float getMaxEncoderX(std::string rover_name);
    float getMaxEncoderY(std::string rover_name);
    float getMinEncoderX(std::string rover_name);
    float getMinEncoderY(std::string rover_name);

    ~MapData();

private:

    std::map<std::string, std::vector< std::pair<float,float> > > gps_rover_path;
    std::map<std::string, std::vector< std::pair<float,float> > >  ekf_rover_path;
    std::map<std::string, std::vector< std::pair<float,float> > >  encoder_rover_path;

    std::map<std::string, std::vector< std::pair<float,float> > >  collection_points;
    std::map<std::string, std::vector< std::pair<float,float> > >  target_locations;

    std::map<std::string, float> max_gps_seen_x;
    std::map<std::string, float> max_gps_seen_y;
    std::map<std::string, float> min_gps_seen_x;
    std::map<std::string, float> min_gps_seen_y;

    std::map<std::string, float> max_encoder_seen_x;
    std::map<std::string, float> max_encoder_seen_y;
    std::map<std::string, float> min_encoder_seen_x;
    std::map<std::string, float> min_encoder_seen_y;

    std::map<std::string, float> max_ekf_seen_x;
    std::map<std::string, float> max_ekf_seen_y;
    std::map<std::string, float> min_ekf_seen_x;
    std::map<std::string, float> min_ekf_seen_y;

    QMutex update_mutex; // To prevent race conditions when the data is being displayed by MapFrame
};

#endif // MAPDATA_H
