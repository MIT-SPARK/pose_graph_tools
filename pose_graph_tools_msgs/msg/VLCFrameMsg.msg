uint32 robot_id
uint32 pose_id
uint32 submap_id
sensor_msgs/Image descriptors_mat          # Descriptor of all keypoints stored as a matrix.
sensor_msgs/PointCloud2 versors            # Bearing vector of each keypoint expressed as a 3D vector.
float32[] depths                           # Depth of each keypoint. The 3D keypoint v can be recovered from the depth d and bearing u by v = d * u / u[2]
geometry_msgs/Pose T_submap_pose
