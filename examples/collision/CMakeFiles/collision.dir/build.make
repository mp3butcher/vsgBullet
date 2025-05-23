# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/xeul/BIGEXT4/SRC/osgbullet-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/xeul/BIGEXT4/SRC/osgbullet-master

# Include any dependencies generated for this target.
include examples/collision/CMakeFiles/collision.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/collision/CMakeFiles/collision.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/collision/CMakeFiles/collision.dir/progress.make

# Include the compile flags for this target's objects.
include examples/collision/CMakeFiles/collision.dir/flags.make

examples/collision/CMakeFiles/collision.dir/collision.cpp.o: examples/collision/CMakeFiles/collision.dir/flags.make
examples/collision/CMakeFiles/collision.dir/collision.cpp.o: examples/collision/collision.cpp
examples/collision/CMakeFiles/collision.dir/collision.cpp.o: examples/collision/CMakeFiles/collision.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/media/xeul/BIGEXT4/SRC/osgbullet-master/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/collision/CMakeFiles/collision.dir/collision.cpp.o"
	cd /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/collision/CMakeFiles/collision.dir/collision.cpp.o -MF CMakeFiles/collision.dir/collision.cpp.o.d -o CMakeFiles/collision.dir/collision.cpp.o -c /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision/collision.cpp

examples/collision/CMakeFiles/collision.dir/collision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/collision.dir/collision.cpp.i"
	cd /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision/collision.cpp > CMakeFiles/collision.dir/collision.cpp.i

examples/collision/CMakeFiles/collision.dir/collision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/collision.dir/collision.cpp.s"
	cd /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision/collision.cpp -o CMakeFiles/collision.dir/collision.cpp.s

# Object files for target collision
collision_OBJECTS = \
"CMakeFiles/collision.dir/collision.cpp.o"

# External object files for target collision
collision_EXTERNAL_OBJECTS =

bin/collision: examples/collision/CMakeFiles/collision.dir/collision.cpp.o
bin/collision: examples/collision/CMakeFiles/collision.dir/build.make
bin/collision: lib/libosgbCollision.a
bin/collision: /usr/lib/x86_64-linux-gnu/libBulletCollision.so
bin/collision: /usr/lib/x86_64-linux-gnu/libLinearMath.so
bin/collision: examples/collision/CMakeFiles/collision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/media/xeul/BIGEXT4/SRC/osgbullet-master/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/collision"
	cd /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/collision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/collision/CMakeFiles/collision.dir/build: bin/collision
.PHONY : examples/collision/CMakeFiles/collision.dir/build

examples/collision/CMakeFiles/collision.dir/clean:
	cd /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision && $(CMAKE_COMMAND) -P CMakeFiles/collision.dir/cmake_clean.cmake
.PHONY : examples/collision/CMakeFiles/collision.dir/clean

examples/collision/CMakeFiles/collision.dir/depend:
	cd /media/xeul/BIGEXT4/SRC/osgbullet-master && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/xeul/BIGEXT4/SRC/osgbullet-master /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision /media/xeul/BIGEXT4/SRC/osgbullet-master /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision /media/xeul/BIGEXT4/SRC/osgbullet-master/examples/collision/CMakeFiles/collision.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/collision/CMakeFiles/collision.dir/depend

