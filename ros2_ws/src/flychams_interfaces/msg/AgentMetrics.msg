std_msgs/Header header

# Agent data
float32 curr_x
float32 curr_y
float32 curr_z
float32 curr_yaw
float32 vel_x
float32 vel_y
float32 vel_z
float32 vel_yaw
float32 goal_x
float32 goal_y
float32 goal_z
float32 goal_yaw

# Position and movement metrics
float32 total_distance_traveled      # Total distance covered by the UAV (m)
float32 current_speed                # Current speed of the UAV (m/s)

# Goal-related metrics
float32 distance_to_goal             # Current distance to assigned goal (m)

# Energy metrics
#float32 battery_level               # TODO: Remaining battery percentage (0-1) 
#float32 power_consumption           # TODO: Current power consumption rate (W)

# Mission metrics
#float32 average_distance_to_targets # TODO: Average distance to assigned targets (m)
#int32 waypoints_reached             # TODO: Number of waypoints successfully reached
float32 time_elapsed                 # Time elapsed since mission start (s)

# Performance metrics
float32 average_speed                # Average speed during mission (m/s)
#float32 tracking_error              # TODO: Error in following planned trajectory (m)
#float32 stability_metric            # Measure of flight stability (0-1)