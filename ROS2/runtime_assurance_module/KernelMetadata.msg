# Total computation time for kernel generator
uint64 processing_time_us
string[] processing_time_detail

# Min value of safety map (current)
float64 state_safeness
float64 state_safeness_current

# Min distance of cartesian safety maps (current)
float64 min_distance_to_obstacles_current

# Min distance of cartesian safety maps along backup trajectory
float64 min_distance_to_obstacles_predicted
bool has_predicted

# Failsafe trajectories data
FailsafeTrajectories failsafe_trajectories
