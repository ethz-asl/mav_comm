Header header

# body-frame: x forward, y left, z up

# rotation convention (z-y'-x''):
# yaw rotates around fixed frame's z axis
# pitch rotates around new y-axis (y')
# roll rotates around new x-axis (x'')

float32 roll      # roll angle [rad]
float32 pitch     # pitch angle  [rad]
float32 yaw_rate  # yaw rate around z-axis [rad/s]
float32 thrust    # thrust [N]
