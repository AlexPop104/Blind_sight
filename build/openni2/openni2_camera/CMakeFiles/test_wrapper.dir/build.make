# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alex/Blind_sight/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alex/Blind_sight/build

# Include any dependencies generated for this target.
include openni2/openni2_camera/CMakeFiles/test_wrapper.dir/depend.make

# Include the progress variables for this target.
include openni2/openni2_camera/CMakeFiles/test_wrapper.dir/progress.make

# Include the compile flags for this target's objects.
include openni2/openni2_camera/CMakeFiles/test_wrapper.dir/flags.make

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/flags.make
openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o: /home/alex/Blind_sight/src/openni2/openni2_camera/test/test_wrapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Blind_sight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o -c /home/alex/Blind_sight/src/openni2/openni2_camera/test/test_wrapper.cpp

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.i"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Blind_sight/src/openni2/openni2_camera/test/test_wrapper.cpp > CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.i

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.s"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Blind_sight/src/openni2/openni2_camera/test/test_wrapper.cpp -o CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.s

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.requires:

.PHONY : openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.requires

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.provides: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.requires
	$(MAKE) -f openni2/openni2_camera/CMakeFiles/test_wrapper.dir/build.make openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.provides.build
.PHONY : openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.provides

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.provides.build: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o


# Object files for target test_wrapper
test_wrapper_OBJECTS = \
"CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o"

# External object files for target test_wrapper
test_wrapper_EXTERNAL_OBJECTS =

/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/build.make
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /home/alex/Blind_sight/devel/lib/libopenni2_wrapper.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libcamera_info_manager.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libcamera_calibration_parsers.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libimage_transport.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libmessage_filters.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libnodeletlib.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libbondcpp.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libclass_loader.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/libPocoFoundation.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libdl.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libroslib.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/librospack.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libroscpp.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/librosconsole.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/librostime.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /opt/ros/melodic/lib/libcpp_common.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/Blind_sight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_wrapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
openni2/openni2_camera/CMakeFiles/test_wrapper.dir/build: /home/alex/Blind_sight/devel/lib/openni2_camera/test_wrapper

.PHONY : openni2/openni2_camera/CMakeFiles/test_wrapper.dir/build

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/requires: openni2/openni2_camera/CMakeFiles/test_wrapper.dir/test/test_wrapper.cpp.o.requires

.PHONY : openni2/openni2_camera/CMakeFiles/test_wrapper.dir/requires

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/clean:
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && $(CMAKE_COMMAND) -P CMakeFiles/test_wrapper.dir/cmake_clean.cmake
.PHONY : openni2/openni2_camera/CMakeFiles/test_wrapper.dir/clean

openni2/openni2_camera/CMakeFiles/test_wrapper.dir/depend:
	cd /home/alex/Blind_sight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Blind_sight/src /home/alex/Blind_sight/src/openni2/openni2_camera /home/alex/Blind_sight/build /home/alex/Blind_sight/build/openni2/openni2_camera /home/alex/Blind_sight/build/openni2/openni2_camera/CMakeFiles/test_wrapper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openni2/openni2_camera/CMakeFiles/test_wrapper.dir/depend

