
std_msgs/UInt16MultiArray tags
sensor_msgs/Image image

# Each polygon object will contain 4 corners
# of an april tag in no particular order
# in the points array
# ex: polygon[0].points[0].x polygon[0].points[0].y
# The index corresponds to the april tag, so the 0th index
# will be the same april tag as the 0th index in the center 
# polygon object
geometry_msgs/Polygon[] corners

# Contains the center coordinates for detected
# in the points array within the polygon object
# ex: points[0].x points[0].y
geometry_msgs/Polygon   centers

