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

# Utility rule file for openni2_camera_generate_messages_cpp.

# Include the progress variables for this target.
include openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/progress.make

openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp: /home/alex/Blind_sight/devel/include/openni2_camera/GetSerial.h


/home/alex/Blind_sight/devel/include/openni2_camera/GetSerial.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/alex/Blind_sight/devel/include/openni2_camera/GetSerial.h: /home/alex/Blind_sight/src/openni2/openni2_camera/srv/GetSerial.srv
/home/alex/Blind_sight/devel/include/openni2_camera/GetSerial.h: /opt/ros/melodic/share/gencpp/msg.h.template
/home/alex/Blind_sight/devel/include/openni2_camera/GetSerial.h: /opt/ros/melodic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alex/Blind_sight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from openni2_camera/GetSerial.srv"
	cd /home/alex/Blind_sight/src/openni2/openni2_camera && /home/alex/Blind_sight/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/alex/Blind_sight/src/openni2/openni2_camera/srv/GetSerial.srv -p openni2_camera -o /home/alex/Blind_sight/devel/include/openni2_camera -e /opt/ros/melodic/share/gencpp/cmake/..

openni2_camera_generate_messages_cpp: openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp
openni2_camera_generate_messages_cpp: /home/alex/Blind_sight/devel/include/openni2_camera/GetSerial.h
openni2_camera_generate_messages_cpp: openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/build.make

.PHONY : openni2_camera_generate_messages_cpp

# Rule to build all files generated by this target.
openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/build: openni2_camera_generate_messages_cpp

.PHONY : openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/build

openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/clean:
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && $(CMAKE_COMMAND) -P CMakeFiles/openni2_camera_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/clean

openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/depend:
	cd /home/alex/Blind_sight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Blind_sight/src /home/alex/Blind_sight/src/openni2/openni2_camera /home/alex/Blind_sight/build /home/alex/Blind_sight/build/openni2/openni2_camera /home/alex/Blind_sight/build/openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openni2/openni2_camera/CMakeFiles/openni2_camera_generate_messages_cpp.dir/depend

