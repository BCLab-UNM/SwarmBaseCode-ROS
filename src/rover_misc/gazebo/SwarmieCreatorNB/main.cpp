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

bool copyDir(
        boost::filesystem::path const & source,
        boost::filesystem::path const & destination
        ) {
    namespace fs = boost::filesystem;
    try {
        // Check whether the function call is valid
        if (
                !fs::exists(source) ||
                !fs::is_directory(source)
                ) {
            std::cerr << "Source directory " << source.string()
                    << " does not exist or is not a directory." << '\n'
                    ;
            return false;
        }
        if (fs::exists(destination)) {
            std::cerr << "Destination directory " << destination.string()
                    << " already exists." << '\n'
                    ;
            return false;
        }
        // Create the destination directory
        if (!fs::create_directory(destination)) {
            std::cerr << "Unable to create destination directory"
                    << destination.string() << '\n'
                    ;
            return false;
        }
    } catch (fs::filesystem_error const & e) {
        std::cerr << e.what() << '\n';
        return false;
    }
    // Iterate through the source directory
    for (
            fs::directory_iterator file(source);
            file != fs::directory_iterator(); ++file
            ) {
        try {
            fs::path current(file->path());
            if (fs::is_directory(current)) {
                // Found directory: Recursion
                if (
                        !copyDir(
                        current,
                        destination / current.filename()
                        )
                        ) {
                    return false;
                }
            } else {
                // Found file: Copy
                fs::copy_file(
                        current,
                        destination / current.filename()
                        );
            }
        } catch (fs::filesystem_error const & e) {
            std::cerr << e.what() << '\n';
        }
    }
    return true;
}

int main(int argc, char*argv[]) {
    char host[128];
    string robotName;
    string colorCode;
    string line;
    string colorLine;
    string pathNameModelConfig;
    string pathNameModelSdf;
    string meshesPath;
    string colorPath;
    ifstream modelConfig("robotTemplate/model.config");
    ifstream modelSdf("robotTemplate/model.sdf");
    ofstream configOutput;
    ofstream sdfOutput;
    ofstream colorOutput;
    string newColor;
    char response;

    gethostname(host, sizeof (host));
    string hostname(host);

    if (argc == 1) {
        cout << "No rover name selected. Default name is: " << hostname << endl;
        robotName = hostname;
    } else {
        robotName = argv[1];
    }

    if (argc >= 3) {
        colorCode = argv[2];
    } else {
        colorCode = "FlatBlack";
        cout << "No rover color selected. Default color is: FlatBlack" << endl;
    }

    if (boost::iequals(colorCode, "Grey") || boost::iequals(colorCode, "Gray")) {
        newColor = "Grey";
    } else if (boost::iequals(colorCode, "White")) {
        newColor = "White";
    } else if (boost::iequals(colorCode, "FlatBlack")) {
        newColor = "FlatBlack";
    } else if (boost::iequals(colorCode, "Black")) {
        newColor = "Black";
    } else if (boost::iequals(colorCode, "Red")) {
        newColor = "Red";
    } else if (boost::iequals(colorCode, "Green")) {
        newColor = "Green";
    } else if (boost::iequals(colorCode, "Blue")) {
        newColor = "Blue";
    } else if (boost::iequals(colorCode, "Yellow")) {
        newColor = "Yellow";
    } else if (boost::iequals(colorCode, "Purple")) {
        newColor = "Purple";
    } else if (boost::iequals(colorCode, "Turquoise")) {
        newColor = "Turquoise";
    } else if (boost::iequals(colorCode, "Orange")) {
        newColor = "Orange";
    } else if (boost::iequals(colorCode, "Gold")) {
        newColor = "Gold";
    } else {
        cout << "No recognized rover color selected. Default color is: FlatBlack" << endl;
        newColor = "FlatBlack";
    }

    string dirLocal = "newRobots/" + robotName;
    if (boost::filesystem::exists(dirLocal)) {
        cout << "Directory already exists" << endl;
        cout << "Overwrite existing directory? [y/n]" << endl;
        cin >> response;
        if (response == 'y'){
            cout << "Will overwrite directory" << endl;
        }
        else if (response == 'n'){
            cout << "Will not delete existing directory." << endl;
            cout << "Exiting." << endl;
            return 0;
        }
        else {
            cout << "Command not recognized. Will not delete existing directory" << endl;
            cout << "Exiting." << endl;
            return 0;
        }
        
    } else {
        boost::filesystem::create_directory(dirLocal);
    }

    pathNameModelConfig = "newRobots/" + robotName + "/model.config";
    pathNameModelSdf = "newRobots/" + robotName + "/tmp.sdf";
    meshesPath = "newRobots/" + robotName + "/meshes";
    colorPath = "newRobots/" + robotName + "/model.sdf";

    copyDir("robotTemplate/meshes", meshesPath);

    configOutput.open(pathNameModelConfig.c_str());

    if (modelConfig.is_open()) {
        while (getline(modelConfig, line)) {
            configOutput << ReplaceString(line, "TEMPLATE", robotName) << endl;

        }
        modelConfig.close();
    }

    sdfOutput.open(pathNameModelSdf.c_str());
    if (modelSdf.is_open()) {
        while (getline(modelSdf, line)) {
            sdfOutput << ReplaceString(line, "TEMPLATE", robotName) << endl;
        }

        sdfOutput.close();
    }

    if (!colorCode.empty()) {
        colorOutput.open(colorPath.c_str());
        ifstream tmpSdf(pathNameModelSdf.c_str());
        if (tmpSdf.is_open()) {
            while (getline(tmpSdf, line)) {
                colorOutput << ReplaceString(line, "ROBOTCOLOR", newColor) << endl;
            }
        }
        colorOutput.close();
        remove(pathNameModelSdf.c_str());

    }

    cout << "Created " << robotName << " in newRobots/" << endl;

    return 0;
}
