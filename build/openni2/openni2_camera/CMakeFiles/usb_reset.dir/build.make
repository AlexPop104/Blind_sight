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
include openni2/openni2_camera/CMakeFiles/usb_reset.dir/depend.make

# Include the progress variables for this target.
include openni2/openni2_camera/CMakeFiles/usb_reset.dir/progress.make

# Include the compile flags for this target's objects.
include openni2/openni2_camera/CMakeFiles/usb_reset.dir/flags.make

openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o: openni2/openni2_camera/CMakeFiles/usb_reset.dir/flags.make
openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o: /home/alex/Blind_sight/src/openni2/openni2_camera/src/usb_reset.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Blind_sight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/usb_reset.dir/src/usb_reset.c.o   -c /home/alex/Blind_sight/src/openni2/openni2_camera/src/usb_reset.c

openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/usb_reset.dir/src/usb_reset.c.i"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/alex/Blind_sight/src/openni2/openni2_camera/src/usb_reset.c > CMakeFiles/usb_reset.dir/src/usb_reset.c.i

openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/usb_reset.dir/src/usb_reset.c.s"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/alex/Blind_sight/src/openni2/openni2_camera/src/usb_reset.c -o CMakeFiles/usb_reset.dir/src/usb_reset.c.s

openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.requires:

.PHONY : openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.requires

openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.provides: openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.requires
	$(MAKE) -f openni2/openni2_camera/CMakeFiles/usb_reset.dir/build.make openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.provides.build
.PHONY : openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.provides

openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.provides.build: openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o


# Object files for target usb_reset
usb_reset_OBJECTS = \
"CMakeFiles/usb_reset.dir/src/usb_reset.c.o"

# External object files for target usb_reset
usb_reset_EXTERNAL_OBJECTS =

/home/alex/Blind_sight/devel/lib/openni2_camera/usb_reset: openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o
/home/alex/Blind_sight/devel/lib/openni2_camera/usb_reset: openni2/openni2_camera/CMakeFiles/usb_reset.dir/build.make
/home/alex/Blind_sight/devel/lib/openni2_camera/usb_reset: openni2/openni2_camera/CMakeFiles/usb_reset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/Blind_sight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable /home/alex/Blind_sight/devel/lib/openni2_camera/usb_reset"
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/usb_reset.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
openni2/openni2_camera/CMakeFiles/usb_reset.dir/build: /home/alex/Blind_sight/devel/lib/openni2_camera/usb_reset

.PHONY : openni2/openni2_camera/CMakeFiles/usb_reset.dir/build

openni2/openni2_camera/CMakeFiles/usb_reset.dir/requires: openni2/openni2_camera/CMakeFiles/usb_reset.dir/src/usb_reset.c.o.requires

.PHONY : openni2/openni2_camera/CMakeFiles/usb_reset.dir/requires

openni2/openni2_camera/CMakeFiles/usb_reset.dir/clean:
	cd /home/alex/Blind_sight/build/openni2/openni2_camera && $(CMAKE_COMMAND) -P CMakeFiles/usb_reset.dir/cmake_clean.cmake
.PHONY : openni2/openni2_camera/CMakeFiles/usb_reset.dir/clean

openni2/openni2_camera/CMakeFiles/usb_reset.dir/depend:
	cd /home/alex/Blind_sight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Blind_sight/src /home/alex/Blind_sight/src/openni2/openni2_camera /home/alex/Blind_sight/build /home/alex/Blind_sight/build/openni2/openni2_camera /home/alex/Blind_sight/build/openni2/openni2_camera/CMakeFiles/usb_reset.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openni2/openni2_camera/CMakeFiles/usb_reset.dir/depend

