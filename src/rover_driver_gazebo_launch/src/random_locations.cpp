/* 
 * File:   randomLocationsMain.cpp
 * Author: gmontague
 *
 * Created on September 12, 2014, 10:33 AM
 */
#include <iomanip>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>
#include <boost/lexical_cast.hpp>
using namespace std;
ofstream randomNumbers("randomNumbers.txt");
float absX;
float absY;

struct Coordinate {
    float x, y;
};

std::vector<Coordinate> randomLocations;
std::vector<Coordinate> tmpLocation;

Coordinate makePoint();
void generateCoords();

void writeToFile();

int main(int argc, char*argv[]) {
    generateCoords();
    writeToFile();

    return 0;
}

Coordinate makePoint() {
    float x = static_cast<float> (rand()) / static_cast<float> (RAND_MAX / 19) - 9;
    float y = static_cast<float> (rand()) / static_cast<float> (RAND_MAX / 19) - 9;
    Coordinate tmpCoord = {x, y};
    return tmpCoord;
}

void generateCoords() {
    srand(unsigned(time(NULL)));
    int errorCount = 0;
    randomLocations.erase(randomLocations.begin(), randomLocations.end());
    randomLocations.push_back(makePoint());
    for (int i = 1; i < 100000; i++) {
        errorCount = 0;
        tmpLocation.erase(tmpLocation.begin(), tmpLocation.end());
        tmpLocation.push_back(makePoint());
        for (int j = 0; j < randomLocations.size(); j++) {
            absX = abs(tmpLocation[0].x - randomLocations[j].x);
            absY = abs(tmpLocation[0].y - randomLocations[j].y);
            if (absX < 0.65 && absY < 0.65) {
                errorCount++;
            } else {
            }
        }
        if (errorCount == 0) {
            randomLocations.push_back(tmpLocation[0]);
        }

        tmpLocation.erase(tmpLocation.begin(), tmpLocation.end());
    }
}

void writeToFile() {
    cout << randomLocations.size() << endl;
    for (int i = 0; i < 500; i++) {
        if (randomLocations[i].x != 0 && randomLocations[i].y != 0) {
            randomNumbers << setprecision(2) << fixed << randomLocations[i].x << " " << randomLocations[i].y << endl;
        }
    }
}