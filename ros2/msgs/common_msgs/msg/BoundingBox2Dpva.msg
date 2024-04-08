# 2D bounding box projected to camera image
# (xmin,ymin)--> +---------+
#                |         |
#                |         |
#                |         |
#                +---------+ <--(xmax,ymax)
int32                 xmin
int32                 ymin
int32                 xmax
int32                 ymax
int32                 label_id
float32               confidence
# radar track projected on image_rect
int32                 radar_track_x
int32                 radar_track_y
# position, velocity (linear) and acceleration (linear) in the radar frame
bool                  pva_valid
geometry_msgs/Point   position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
