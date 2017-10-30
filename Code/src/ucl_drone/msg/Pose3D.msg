
# This represents an estimate of a position and velocity in 3D space.
# The pose in this message should be specified in an absolute coordinate frame.

# ucl definition of a pose message.
# TODO: unites?
# Also add velocities for the controller purpose.
# TODO: why exclude acceleration?

Header header

float64 x
float64 y
float64 z

#float64 quatX
#float64 quatY
#float64 quatZ
#float64 quatW

float64 rotX
float64 rotY
float64 rotZ

float64 xvel
float64 yvel
float64 zvel

float64 rotXvel
float64 rotYvel
float64 rotZvel
