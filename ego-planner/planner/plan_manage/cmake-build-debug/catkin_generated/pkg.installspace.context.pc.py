# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "plan_env;path_searching;bspline_opt;traj_utils;message_runtime;mpc_controller".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lego_planner".split(';') if "-lego_planner" != "" else []
PROJECT_NAME = "ego_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
