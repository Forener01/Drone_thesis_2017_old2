
# This represents an estimate of a position and velocity in 3D space.
# The pose in this message should be specified in an absolute coordinate frame.

# ucl definition of a desired pose message.

Header header

float64 x
float64 y
float64 z

float64 rotZ

bool landAndStop
bool takeoffAndStart
