# Single detected object from YOLO
std_msgs/Header header

# Object classification
string class_name          # e.g., "person", "box", "pallet"
int32 class_id             # COCO class ID
float32 confidence         # Detection confidence (0.0 - 1.0)

# 2D bounding box in image coordinates
float32[] bbox_2d          # [x1, y1, x2, y2] in pixels

# 3D position in world frame (after transformation)
geometry_msgs/Point position_3d

