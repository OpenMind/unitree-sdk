# Match score information for lidar localization
# Contains pose data and matching quality metrics

# Header for timestamp and frame information
std_msgs/Header header

# Robot pose in map frame
geometry_msgs/Pose pose

# Match score metrics
int32 match_score          # Raw matching score
float32 quality_percent    # Match quality as percentage (0-100)
int32 num_points          # Number of scan points used for matching
