std_msgs/Header header
string class_name
uint32 top_left_x        # X coordinate of bounding box
uint32 top_left_y        # Y coordinate of bounding box
uint32 width    # Width of bounding box
uint32 height   # Height of bounding box
float32 class_prob #confidence of detection from yolo
uint32 image_width #Width of the image def - 640
uint32 image_height #Height of the image def - 480