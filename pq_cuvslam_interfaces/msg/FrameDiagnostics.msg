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

# Health check of the odometry (impl/tracking_health.hpp): 0 ok, 1 degraded (few landmarks),
# 2 rejected (implausible speed or rotation against the gyroscope), 3 lost. Degraded reasons:
# few_landmarks, vio (VIO speed above the odometry speed, see vio_mismatch_mps).
uint8 health
string health_reasons    # e.g. speed+gyro
float32 speed_mps        # speed of the odometry increment; NaN without increment
float32 gyro_error_deg   # rotation of the increment against the gyroscope; NaN without gyroscope

# State of the visual-inertial odometry (cuVSLAM >= 17 with IMU fusion; otherwise
# imu_state_valid is false and the values are NaN).
bool imu_state_valid
bool vio_initialized     # gravity estimated: visual-inertial initialization done
float32 vio_speed_mps    # speed estimated by the VIO (norm of its velocity)
float32 vio_mismatch_mps # VIO speed - median odometry speed of the last 0.5 s (health "vio")
float32[3] gyro_bias     # rad/s, IMU frame
float32[3] acc_bias      # m/s^2, IMU frame

StreamStats[] streams    # one per input image of the frame
