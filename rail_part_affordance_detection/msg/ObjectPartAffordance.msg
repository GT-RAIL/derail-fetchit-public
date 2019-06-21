std_msgs/Header header
uint8 object_class
float32 detection_score
# [x1, y1, x2, y2] specifying top-left and bottom-right corners
uint16[] bounding_box
uint32 height
uint32 width
uint8[] affordance_mask