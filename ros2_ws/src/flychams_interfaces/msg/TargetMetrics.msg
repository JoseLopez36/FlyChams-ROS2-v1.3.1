std_msgs/Header header

# Target data
float32 curr_x
float32 curr_y
float32 curr_z

# Position and movement metrics
float32 total_distance_traveled     # Total distance covered by target (m)
#float32 average_speed              # TODO: Average speed over time window (m/s)

# Estimation metrics
#float32 estimated_x                # TODO: Estimated x position (m)
#float32 estimated_y                # TODO: Estimated y position (m)