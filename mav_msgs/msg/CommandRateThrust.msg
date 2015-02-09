Header header

# Message should be specified in the body coordinate frame.

# We use the coordinate frames with the following convention:
#   x: forward
#   y: left
#   z: up

float64 roll_rate    # roll rate around body x-axis [rad/s]
float64 pitch_rate   # pitch rate around body y-axis [rad/s]
float64 yaw_rate     # yaw rate around body z-axis [rad/s]
float64 thrust       # thrust [N]
