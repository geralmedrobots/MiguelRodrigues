# This message contains wheel tick data from the Roboteq motor driver configured as differential drive platform,
# typically used for odometry purposes.
# Add header

std_msgs/Header header    # header
int32 right_ticks         # odometer count of the right motor
int32 left_ticks          # odometer count of the left motor
float32 right_ticks_norm  # Normalized right ticks  ticks/total_ticks_turn
float32 left_ticks_norm   # Normalized left ticks ticks/total_ticks_turn