# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ego_planner: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iego_planner:/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ego_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" NAME_WE)
add_custom_target(_ego_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ego_planner" "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" NAME_WE)
add_custom_target(_ego_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ego_planner" "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ego_planner
)
_generate_msg_cpp(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ego_planner
)

### Generating Services

### Generating Module File
_generate_module_cpp(ego_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ego_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ego_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ego_planner_generate_messages ego_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_cpp _ego_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_cpp _ego_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ego_planner_gencpp)
add_dependencies(ego_planner_gencpp ego_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ego_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ego_planner
)
_generate_msg_eus(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ego_planner
)

### Generating Services

### Generating Module File
_generate_module_eus(ego_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ego_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ego_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ego_planner_generate_messages ego_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_eus _ego_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_eus _ego_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ego_planner_geneus)
add_dependencies(ego_planner_geneus ego_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ego_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ego_planner
)
_generate_msg_lisp(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ego_planner
)

### Generating Services

### Generating Module File
_generate_module_lisp(ego_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ego_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ego_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ego_planner_generate_messages ego_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_lisp _ego_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_lisp _ego_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ego_planner_genlisp)
add_dependencies(ego_planner_genlisp ego_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ego_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ego_planner
)
_generate_msg_nodejs(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ego_planner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ego_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ego_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ego_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ego_planner_generate_messages ego_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_nodejs _ego_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_nodejs _ego_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ego_planner_gennodejs)
add_dependencies(ego_planner_gennodejs ego_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ego_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ego_planner
)
_generate_msg_py(ego_planner
  "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ego_planner
)

### Generating Services

### Generating Module File
_generate_module_py(ego_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ego_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ego_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ego_planner_generate_messages ego_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/Bspline.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_py _ego_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/msg/DataDisp.msg" NAME_WE)
add_dependencies(ego_planner_generate_messages_py _ego_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ego_planner_genpy)
add_dependencies(ego_planner_genpy ego_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ego_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ego_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ego_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ego_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ego_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ego_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ego_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ego_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ego_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ego_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ego_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ego_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ego_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ego_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ego_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ego_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ego_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ego_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ego_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ego_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ego_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ego_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
