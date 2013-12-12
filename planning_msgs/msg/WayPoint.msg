Header header
float64[] x       # holds motion primitive for the x-axis
float64[] y       # holds motion primitive for the y-axis
float64[] z       # holds motion primitive for the z-axis
float64[] yaw     # holds motion primitive for yaw
int32 type        # defines the type of the waypoint
float64 time      # time for that waypoint. If it is a "real" point, this is the time to stay on that point. In case of a polynomial, it is the time [0...time] for which the polynomial is valid
