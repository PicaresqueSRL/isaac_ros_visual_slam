# Diagnostics of one processed frame of pq_cuvslam (topic visual_slam/diagnostics/frame).

std_msgs/Header header   # stamp of the frame, frame_id = map frame

string mode              # tracking mode: Multicamera, VIO (IMU fusion), RGBD
string profile           # camera layout profile, empty without profile
bool tracked             # visual odometry succeeded for this frame

# Times, in milliseconds.
float32 track_time_ms    # cuVSLAM Track() call
float32 callback_time_ms # whole frame processing in the node
float32 latency_ms       # node clock at publication - frame stamp; NaN in a bag replay
float32 sync_offset_ms   # capture time difference between devices of the frame; NaN if none

# cuVSLAM tracking state (zero when the export of observations/landmarks is disabled).
string[] camera_names
uint32[] observations    # tracked 2D observations, per camera (same order as camera_names)
uint32 landmarks         # landmarks of the last frame

# SLAM (false when SLAM is disabled).
bool loop_closure        # loop closure detected in this frame
bool pose_graph_optimized

StreamStats[] streams    # one per input image of the frame
