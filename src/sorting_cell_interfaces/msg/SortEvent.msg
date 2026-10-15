# One pick-and-place attempt, published by the task node when it finishes.
uint8 RESULT_SORTED = 0          # placed into a bin
uint8 RESULT_GRASP_FAILED = 1    # nothing (or the wrong thing) in the gripper
uint8 RESULT_PLANNING_FAILED = 2 # MoveIt could not plan / execute a motion
uint8 RESULT_LOST = 3            # part dropped or left the pick zone

builtin_interfaces/Time stamp
uint32 cycle                     # running attempt counter
string part_id                   # tracker id from perception
string predicted_class           # e.g. "cube", "hex_nut_defect"
float32 confidence
string target_bin
string attached_part             # Gazebo part actually held (for evaluation)
uint8 result
string failure_reason
uint8 attempts                   # grasp attempts used
float32 cycle_time               # target selected -> part released [s]
geometry_msgs/Pose pick_pose     # estimated grasp pose in world
