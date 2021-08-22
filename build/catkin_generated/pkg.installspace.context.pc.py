# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rqt_gui;rqt_gui_cpp;image_transport;sensor_msgs;cv_bridge;geometry_msgs;rviz;rqt_rviz".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lborealis_gui".split(';') if "-lborealis_gui" != "" else []
PROJECT_NAME = "borealis_gui"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"
