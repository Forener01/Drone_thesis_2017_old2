
(cl:in-package :asdf)

(defsystem "ucl_drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :ardrone_autonomy-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DroneRole" :depends-on ("_package_DroneRole"))
    (:file "_package_DroneRole" :depends-on ("_package"))
    (:file "TargetDetected" :depends-on ("_package_TargetDetected"))
    (:file "_package_TargetDetected" :depends-on ("_package"))
    (:file "KeyPoint" :depends-on ("_package_KeyPoint"))
    (:file "_package_KeyPoint" :depends-on ("_package"))
    (:file "ProcessedImageMsg" :depends-on ("_package_ProcessedImageMsg"))
    (:file "_package_ProcessedImageMsg" :depends-on ("_package"))
    (:file "Pose3D" :depends-on ("_package_Pose3D"))
    (:file "_package_Pose3D" :depends-on ("_package"))
    (:file "cellUpdate" :depends-on ("_package_cellUpdate"))
    (:file "_package_cellUpdate" :depends-on ("_package"))
    (:file "PoseRef" :depends-on ("_package_PoseRef"))
    (:file "_package_PoseRef" :depends-on ("_package"))
    (:file "Strategy" :depends-on ("_package_Strategy"))
    (:file "_package_Strategy" :depends-on ("_package"))
    (:file "StrategyMsg" :depends-on ("_package_StrategyMsg"))
    (:file "_package_StrategyMsg" :depends-on ("_package"))
    (:file "DroneRoles" :depends-on ("_package_DroneRoles"))
    (:file "_package_DroneRoles" :depends-on ("_package"))
  ))