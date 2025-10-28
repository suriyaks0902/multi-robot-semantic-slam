# Semantic map containing detected object locations in world frame
std_msgs/Header header

# Parallel arrays containing object information
string[] object_ids          # Unique ID for each object
string[] class_names         # Object class (e.g., "box", "person")
geometry_msgs/Point[] positions  # 3D position in map frame
float32[] confidences        # Detection confidence (0.0 - 1.0)

