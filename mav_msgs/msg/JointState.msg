# Message containing all reference info for a joint of an aerial manipulator

string frame_id
string parent_id

geometry_msgs/Transform transform           # transform of frame_id w.r.t. parent_id
geometry_msgs/Twist twist                   # linear and angular velocity
geometry_msgs/Twist acceleration            # linear and angular acceleration
geometry_msgs/Twist jerk                    # linear and angular jerk
geometry_msgs/Wrench wrench                 # Reference interaction wrench of this joint
geometry_msgs/Wrench wrench_derivative      # and its time derivative