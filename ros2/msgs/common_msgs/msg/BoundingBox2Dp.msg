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
# position in camera frame
geometry_msgs/Point32 position