std_msgs/Header header

# Cluster data
float32 curr_center_x
float32 curr_center_y
float32 curr_center_z
float32 curr_radius

# Position and movement metrics
float32 total_distance_traveled      # Total distance covered by cluster (m)

# Cluster composition metrics
#int32 num_targets                   # TODO: Number of targets in cluster
#float32 cluster_density             # TODO: Density of targets within cluster radius
#float32 cluster_radius              # TODO: Radius of the enclosing circle/sphere
#float32 cluster_area                # TODO: Area covered by the cluster

# Cluster movement metrics
#float32 centroid_average_speed      # TODO: Average speed of cluster centroid
#float32 dispersion                  # TODO: Measure of how spread out targets are from centroid

# Cluster persistence metrics
#float32 persistence_score           # TODO: How stable cluster membership is over time (0-1)
#float32 persistence_time            # TODO: How long cluster has been unchanged (seconds)

# Tracking metrics
#float32 tracking_confidence         # TODO: Confidence in cluster tracking (0-1)