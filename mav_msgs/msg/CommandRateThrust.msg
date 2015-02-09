Header header

# Message should be specified in the body coordinate frame.

# We use the coordinate frames with the following convention:
#   x: forward
#   y: left
#   z: up

geometry_msgs/Vector3 angular_rates  # roll-, pitch-, yaw-rate around body axes [rad/s]
float64 thrust                       # thrust [N]
