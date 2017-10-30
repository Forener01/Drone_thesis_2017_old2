# Controller
- takes inputs from pose_estimation (pose) and path_planning (poseref)
- publishes in cmd_vel and navdata
- function *reguXY*
  - inputs: x_mes/y_mes (pose), x_desired/y_desired (poseref)
  - outputs: xvel_cmd/yvel_cmd
- function *controlLoop*
  - runs regulators from Pose and Poseref
  - publishes the results into cmd_vel
- function *main*
  - create an object "bc" of class "BasicController"
  - launch bc.controlLoop

# Path planning
- takes inputs from pose_estimation (pose) and strategy
- publishes in path_planning (poseref) and mapcell
- function *xy_desired*
  - performs a labyrinth trajectory
- function *advanced_xy_desired*
  - similar to xy_desired
- function *SetRef*
  - loads new positions into "this" pointer
- function *main*
  - create an object "myPath" of class "PathPlanning"
  - publish poseref 
  - hardcode the paths
  
# Pose estimation
- takes inputs from navdata, odometry, pose_visual
- publishes in pose_estimation
- function *poseCopy* (from ardrone_autonomy)
  - assign new values to pose_msg components
- function *poseFusion* (visual_pose not used)
  - assign new values to pose_msg components
- function *queuePoseFusion* (intregation from optical flow)
  - assign new values to pose_msg components
- function *main*
  - create an object "myPose" of class "PoseEstimator"
  - launch FlatTrim and Reset
  - publishes pose 