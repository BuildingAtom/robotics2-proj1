# A basic message for the pose (simplified for 2D)
# Estimates the pose of the robot's origin

Header header       # Reference shenanigans (frame is always world)

float64 x           # x position in world frame in meters (inf if N/A)
float64 y           # y position in world frame in meters (inf if N/A)
float64 theta       # theta orientation in world frame in radians (inf if N/A)

# Row-major 3x3 covariance of the x, y, and theta.
#float64[9]　covariance
