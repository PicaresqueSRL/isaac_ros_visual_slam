# Event of pq_cuvslam (topic visual_slam/events).

uint8 TRACKING_LOST = 1
uint8 TRACKING_RECOVERED = 2
uint8 LOOP_CLOSURE = 3
uint8 CONFIGURED = 4     # node configured: mode and profile in description
uint8 TRACKER_INITIALIZED = 5
uint8 POSE_REJECTED = 6   # first of a run of rejected odometry increments (reasons in description)
uint8 POSE_ACCEPTED = 7   # increments accepted again (number of rejected frames in description)
uint8 TRACKER_RESET = 8   # tracker recreated after repeated rejected or lost frames (reason in description)
uint8 CAMERA_SWITCH = 9   # odometry selector: another tracker selected (odometry_selector_node)
uint8 VIO_INITIALIZED = 10  # visual-inertial initialization done: gravity estimated (cuVSLAM >= 17)

std_msgs/Header header   # stamp of the frame (CONFIGURED: node clock)
uint8 type
string description
