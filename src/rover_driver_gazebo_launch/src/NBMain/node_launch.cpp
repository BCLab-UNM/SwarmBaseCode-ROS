#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

std::string ReplaceString(std::string subject, const std::string& search,
        const std::string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);


        pos += replace.length();
    }
    return subject;
}


int main(int argc, char*argv[]) {
    string robotName = argv[1];
    string line;
    string launchPath;
    ifstream launchConfig("launch_template/driver_template.launch");
    ofstream launchOutput;
    
    launchPath = robotName + ".launch";
    
    
    launchOutput.open(launchPath.c_str());
    if (launchConfig.is_open()) {
        while (getline(launchConfig, line)) {
            launchOutput << ReplaceString(line, "TEMPLATE", robotName) << endl;

        }
        launchConfig.close();
    }
    
    return 0;
}