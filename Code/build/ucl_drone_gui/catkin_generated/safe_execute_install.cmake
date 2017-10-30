execute_process(COMMAND "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone_gui/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone_gui/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
