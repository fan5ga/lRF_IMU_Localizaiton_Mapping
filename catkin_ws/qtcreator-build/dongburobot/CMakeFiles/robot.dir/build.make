# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wangzt/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wangzt/catkin_ws/qtcreator-build

# Include any dependencies generated for this target.
include dongburobot/CMakeFiles/robot.dir/depend.make

# Include the progress variables for this target.
include dongburobot/CMakeFiles/robot.dir/progress.make

# Include the compile flags for this target's objects.
include dongburobot/CMakeFiles/robot.dir/flags.make

dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o: dongburobot/CMakeFiles/robot.dir/flags.make
dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o: /home/wangzt/catkin_ws/src/dongburobot/src/robot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/wangzt/catkin_ws/qtcreator-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o"
	cd /home/wangzt/catkin_ws/qtcreator-build/dongburobot && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/robot.dir/src/robot.cpp.o -c /home/wangzt/catkin_ws/src/dongburobot/src/robot.cpp

dongburobot/CMakeFiles/robot.dir/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/src/robot.cpp.i"
	cd /home/wangzt/catkin_ws/qtcreator-build/dongburobot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/wangzt/catkin_ws/src/dongburobot/src/robot.cpp > CMakeFiles/robot.dir/src/robot.cpp.i

dongburobot/CMakeFiles/robot.dir/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/src/robot.cpp.s"
	cd /home/wangzt/catkin_ws/qtcreator-build/dongburobot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/wangzt/catkin_ws/src/dongburobot/src/robot.cpp -o CMakeFiles/robot.dir/src/robot.cpp.s

dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.requires:
.PHONY : dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.requires

dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.provides: dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.requires
	$(MAKE) -f dongburobot/CMakeFiles/robot.dir/build.make dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.provides.build
.PHONY : dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.provides

dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.provides.build: dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o

# Object files for target robot
robot_OBJECTS = \
"CMakeFiles/robot.dir/src/robot.cpp.o"

# External object files for target robot
robot_EXTERNAL_OBJECTS =

devel/lib/dongburobot/robot: dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libtf.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libtf2_ros.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libactionlib.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libtf2.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libcv_bridge.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_videostab.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_superres.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_stitching.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_contrib.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libimage_transport.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libmessage_filters.so
devel/lib/dongburobot/robot: /usr/lib/libtinyxml.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libclass_loader.so
devel/lib/dongburobot/robot: /usr/lib/libPocoFoundation.so
devel/lib/dongburobot/robot: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libroscpp.so
devel/lib/dongburobot/robot: /usr/lib/libboost_signals-mt.so
devel/lib/dongburobot/robot: /usr/lib/libboost_filesystem-mt.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/librosconsole.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/librosconsole_log4cxx.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/librosconsole_backend_interface.so
devel/lib/dongburobot/robot: /usr/lib/liblog4cxx.so
devel/lib/dongburobot/robot: /usr/lib/libboost_regex-mt.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libxmlrpcpp.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libroslib.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libroscpp_serialization.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/librostime.so
devel/lib/dongburobot/robot: /usr/lib/libboost_date_time-mt.so
devel/lib/dongburobot/robot: /usr/lib/libboost_system-mt.so
devel/lib/dongburobot/robot: /usr/lib/libboost_thread-mt.so
devel/lib/dongburobot/robot: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libcpp_common.so
devel/lib/dongburobot/robot: /opt/ros/hydro/lib/libconsole_bridge.so
devel/lib/dongburobot/robot: dongburobot/CMakeFiles/robot.dir/build.make
devel/lib/dongburobot/robot: dongburobot/CMakeFiles/robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../devel/lib/dongburobot/robot"
	cd /home/wangzt/catkin_ws/qtcreator-build/dongburobot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dongburobot/CMakeFiles/robot.dir/build: devel/lib/dongburobot/robot
.PHONY : dongburobot/CMakeFiles/robot.dir/build

dongburobot/CMakeFiles/robot.dir/requires: dongburobot/CMakeFiles/robot.dir/src/robot.cpp.o.requires
.PHONY : dongburobot/CMakeFiles/robot.dir/requires

dongburobot/CMakeFiles/robot.dir/clean:
	cd /home/wangzt/catkin_ws/qtcreator-build/dongburobot && $(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean.cmake
.PHONY : dongburobot/CMakeFiles/robot.dir/clean

dongburobot/CMakeFiles/robot.dir/depend:
	cd /home/wangzt/catkin_ws/qtcreator-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wangzt/catkin_ws/src /home/wangzt/catkin_ws/src/dongburobot /home/wangzt/catkin_ws/qtcreator-build /home/wangzt/catkin_ws/qtcreator-build/dongburobot /home/wangzt/catkin_ws/qtcreator-build/dongburobot/CMakeFiles/robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dongburobot/CMakeFiles/robot.dir/depend
