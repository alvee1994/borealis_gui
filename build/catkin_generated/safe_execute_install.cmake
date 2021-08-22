execute_process(COMMAND "/home/lv/ROS/ros-noetic-ws/src/borealis_gui/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/lv/ROS/ros-noetic-ws/src/borealis_gui/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
