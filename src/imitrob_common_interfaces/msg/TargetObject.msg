geometry_msgs/Pose eef_pose  # Grasp pose for the EEF
bool relative  # if true uri must be specified; then (grasp_pose = target_uri.pose * eef_pose)
geometry_msgs/Vector3 size  # AABB of the object (e.g., for gripper opening width)
string target_uri  # ref to target object (eef_pose is ignored if not relative and len(uri) > 0)