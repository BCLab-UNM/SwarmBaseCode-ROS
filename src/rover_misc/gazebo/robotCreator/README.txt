This program, robotCreator, creates the necessary files for a SWARMIE rover to be inserted into the simulation environment, Gazebo. The syntax of the program is:

./robotCreator [NAME] [COLOR]

The [NAME] argument is the name you wish to name the rover. This IS case sensitive and will only accept names that are NOT separated by spaces.The following are NOT acceptable:

"rover one"
"new rover"
"awesome rover"

The following, however, are:

"rover1"
"newRover"
"awesome_rover"

By convention, the Swarmies are named with all lower case letters (e.g. "moe" instead of "MOE" or "Moe")

The [COLOR] argument is what the color of the top plate of the Swarmie will be. This is not case sensitive but the list of accepted colors are:

Grey
White
FlatBlack
Black
Red
Green
Blue
Yellow
Purple
Turquoise
Orange
Gold

If a color from this list is not selected or if there is no color input, then the program will default to the color "FlatBlack"

NOTE: If no name and no color are selected then the program will default to:
[NAME]=hostname (the hostname of the system you are on)
[COLOR]=FlatBlack

NECESSARY FILES AND DIRECTORIES

This program needs the directory "template" in its working directory at runtime. template contains two files and one folder that contain templates for the rover models:

model.sdf
model.config
meshes/
meshes/Swarmie_Camera_V2.dae
meshes/SwarmieChassisBare_V2.stl
meshes/Swarmie_Chassis_V2.dae
meshes/Swarmie_Tire.dae
meshes/Swarmie_Tire_V2.dae
meshes/SwarmieTire_V2.stl
meshes/SwarmieTopPlate_V2.stl
meshes/Swarmie_Top_V2.dae
meshes/SwarmieUS_V2.dae


It also requires the directory "../models/" to exist in the working directory at runtime. This is where the program will drop the newly created directory for the new swarmie.


