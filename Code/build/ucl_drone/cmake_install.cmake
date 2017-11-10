# Install script for directory: /home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucl_drone/msg" TYPE FILE FILES
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/Pose3D.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/TargetDetected.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/PoseRef.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/StrategyMsg.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/DroneRole.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/DroneRoles.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/KeyPoint.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/ProcessedImageMsg.msg"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/msg/cellUpdate.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucl_drone/cmake" TYPE FILE FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone/catkin_generated/installspace/ucl_drone-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/devel/include/ucl_drone")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/devel/share/common-lisp/ros/ucl_drone")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/devel/lib/python2.7/dist-packages/ucl_drone")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/devel/lib/python2.7/dist-packages/ucl_drone")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone/catkin_generated/installspace/ucl_drone.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucl_drone/cmake" TYPE FILE FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone/catkin_generated/installspace/ucl_drone-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucl_drone/cmake" TYPE FILE FILES
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone/catkin_generated/installspace/ucl_droneConfig.cmake"
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone/catkin_generated/installspace/ucl_droneConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucl_drone" TYPE FILE FILES "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

