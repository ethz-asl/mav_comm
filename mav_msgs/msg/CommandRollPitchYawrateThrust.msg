Header header

# We use the coordinate frames with the following convention:
#   x: forward
#   y: left
#   z: up

# rotation convention (z-y'-x''):
# yaw rotates around fixed frame's z axis
# pitch rotates around new y-axis (y')
# roll rotates around new x-axis (x'')

float64 roll      # roll angle [rad]
float64 pitch     # pitch angle  [rad]
float64 yaw_rate  # yaw rate around z-axis [rad/s]
float64 thrust    # thrust [N]
