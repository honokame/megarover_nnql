# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;marker_msgs;message_runtime;mrpt_msgs;nav_msgs;sensor_msgs;std_msgs;stereo_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmrpt_bridge".split(';') if "-lmrpt_bridge" != "" else []
PROJECT_NAME = "mrpt_bridge"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.25"
