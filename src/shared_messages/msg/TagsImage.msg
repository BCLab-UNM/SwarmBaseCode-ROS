
std_msgs/UInt16MultiArray tags
sensor_msgs/Image image

# Contains the coordinates {x,y} for april tag
# detected starting from the top left (TL)
# going counter clockwise
# {TL, TR, BR, BL}
geometry_msgs/Polygon[] corners

# Contains the center coordinates for detected
# april tag {x,y} 
float64[]   center

