# Event of pq_cuvslam (topic visual_slam/events).

uint8 TRACKING_LOST = 1
uint8 TRACKING_RECOVERED = 2
uint8 LOOP_CLOSURE = 3
uint8 CONFIGURED = 4     # node configured: mode and profile in description
uint8 TRACKER_INITIALIZED = 5

std_msgs/Header header   # stamp of the frame (CONFIGURED: node clock)
uint8 type
string description
