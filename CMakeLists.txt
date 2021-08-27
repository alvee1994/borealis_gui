cmake_minimum_required(VERSION 3.0.2)

project(borealis_gui)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS stretchsense_server_action_msgs roscpp rqt_gui rqt_gui_cpp image_transport sensor_msgs geometry_msgs cv_bridge rqt_rviz rviz)
find_package(Boost REQUIRED COMPONENTS program_options)
link_directories(${Boost_LIBRARY_DIRS})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(borealis_gui_SRCS
  src/main_gui.cpp

  src/image_frame/ratio_layouted_frame.cpp
  src/image_frame/image_interactive_layout.cpp

  src/borealis_rviz/rviz.cpp 
  src/borealis_rviz/config_dialog.cpp

  src/borealis_pages/borealis_map_and_drone/borealis_map_and_drone.cpp
  src/borealis_pages/borealis_smart_glove/borealis_smart_glove.cpp
  src/borealis_pages/borealis_smart_glove/smart_glove_client.cpp
)

set(borealis_gui_HDRS
  include/main_gui.h

  include/image_frame/ratio_layouted_frame.h
  include/image_frame/image_interactive_layout.h

  include/borealis_rviz/rviz.h
  include/borealis_rviz/config_dialog.h

  include/borealis_pages/borealis_map_and_drone/borealis_map_and_drone.h
  include/borealis_pages/borealis_smart_glove/borealis_smart_glove.h
  include/borealis_pages/borealis_smart_glove/smart_glove_client.h
  )

set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")

# set(borealis_gui_UIS
#   resource/image_view.ui
# )

set(borealis_gui_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${borealis_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge geometry_msgs rviz rqt_rviz
)
catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(borealis_gui_MOCS ${borealis_gui_HDRS})
else()
  qt4_wrap_cpp(borealis_gui_MOCS ${borealis_gui_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
# if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
#   qt5_wrap_ui(borealis_gui_UIS_H ${borealis_gui_UIS})
# else()
#   qt4_wrap_ui(borealis_gui_UIS_H ${borealis_gui_UIS})
# endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

# FIND_LIBRARY(RQT_RVIZ_LIBRARY rqt_rviz /opt/ros/noetic/lib/)
# message(STATUS "rqt_rviz ${RQT_RVIZ_LIBRARY}")

include_directories(${borealis_gui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${borealis_gui_SRCS} ${borealis_gui_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/borealis_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


