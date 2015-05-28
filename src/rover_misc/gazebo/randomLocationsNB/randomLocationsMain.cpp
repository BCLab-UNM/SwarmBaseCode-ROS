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
using namespace std;

std::vector<int> Num_Array;
int x, y, z;

int main(int argc, char*argv[]) {
    int amount = atoi(argv[1]);
    ofstream randomNumbers;

    srand(unsigned(time(NULL)));

    randomNumbers.open("randomNumbers.txt");

    for (x = 0; x < amount; x++) {
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX/19) - 9;
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX/19) - 9;
        randomNumbers << setprecision(2) << fixed << x << " ";
        randomNumbers << setprecision(2) << fixed << y << endl;
        
    }
    randomNumbers.close();
    return 0;
}