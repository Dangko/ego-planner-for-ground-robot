# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/dango/dango_file/software/clion/CLion-2020.2.4/clion-2020.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/dango/dango_file/software/clion/CLion-2020.2.4/clion-2020.2.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/traj_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/traj_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/traj_server.dir/flags.make

CMakeFiles/traj_server.dir/src/traj_server.cpp.o: CMakeFiles/traj_server.dir/flags.make
CMakeFiles/traj_server.dir/src/traj_server.cpp.o: ../src/traj_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/traj_server.dir/src/traj_server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/traj_server.dir/src/traj_server.cpp.o -c /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/src/traj_server.cpp

CMakeFiles/traj_server.dir/src/traj_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/traj_server.dir/src/traj_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/src/traj_server.cpp > CMakeFiles/traj_server.dir/src/traj_server.cpp.i

CMakeFiles/traj_server.dir/src/traj_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/traj_server.dir/src/traj_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/src/traj_server.cpp -o CMakeFiles/traj_server.dir/src/traj_server.cpp.s

# Object files for target traj_server
traj_server_OBJECTS = \
"CMakeFiles/traj_server.dir/src/traj_server.cpp.o"

# External object files for target traj_server
traj_server_EXTERNAL_OBJECTS =

devel/lib/ego_planner/traj_server: CMakeFiles/traj_server.dir/src/traj_server.cpp.o
devel/lib/ego_planner/traj_server: CMakeFiles/traj_server.dir/build.make
devel/lib/ego_planner/traj_server: /home/dango/dango_file2/graduation_project/auto_exploration/devel/lib/libtraj_utils.so
devel/lib/ego_planner/traj_server: /home/dango/dango_file2/graduation_project/auto_exploration/devel/lib/libpath_searching.so
devel/lib/ego_planner/traj_server: /home/dango/dango_file2/graduation_project/auto_exploration/devel/lib/libbspline_opt.so
devel/lib/ego_planner/traj_server: /home/dango/dango_file2/graduation_project/auto_exploration/devel/lib/libplan_env.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/ego_planner/traj_server: /home/dango/dango_file2/graduation_project/auto_exploration/devel/lib/libmpc_controller.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libtf.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libtf2_ros.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libactionlib.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libroscpp.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libtf2.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/librosconsole.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/librostime.so
devel/lib/ego_planner/traj_server: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/ego_planner/traj_server: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/ego_planner/traj_server: CMakeFiles/traj_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/ego_planner/traj_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/traj_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/traj_server.dir/build: devel/lib/ego_planner/traj_server

.PHONY : CMakeFiles/traj_server.dir/build

CMakeFiles/traj_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/traj_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/traj_server.dir/clean

CMakeFiles/traj_server.dir/depend:
	cd /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug /home/dango/dango_file2/graduation_project/auto_exploration/src/ego-planner-master/planner/plan_manage/cmake-build-debug/CMakeFiles/traj_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/traj_server.dir/depend
