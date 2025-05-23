# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Available install components are: \"Unspecified\" \"libosgbbullet-dev\" \"libosgworks\" \"libvsgbbullet\" \"osgbullet\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components
.PHONY : list_install_components/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local/fast

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /media/xeul/BIGEXT4/SRC/osgbullet-master/CMakeFiles /media/xeul/BIGEXT4/SRC/osgbullet-master//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /media/xeul/BIGEXT4/SRC/osgbullet-master/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named osgbpp

# Build rule for target.
osgbpp: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 osgbpp
.PHONY : osgbpp

# fast build rule for target.
osgbpp/fast:
	$(MAKE) $(MAKESILENT) -f applications/osgbpp/CMakeFiles/osgbpp.dir/build.make applications/osgbpp/CMakeFiles/osgbpp.dir/build
.PHONY : osgbpp/fast

#=============================================================================
# Target rules for targets named BasicDemo

# Build rule for target.
BasicDemo: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 BasicDemo
.PHONY : BasicDemo

# fast build rule for target.
BasicDemo/fast:
	$(MAKE) $(MAKESILENT) -f examples/basicdemo/CMakeFiles/BasicDemo.dir/build.make examples/basicdemo/CMakeFiles/BasicDemo.dir/build
.PHONY : BasicDemo/fast

#=============================================================================
# Target rules for targets named centerofmass

# Build rule for target.
centerofmass: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 centerofmass
.PHONY : centerofmass

# fast build rule for target.
centerofmass/fast:
	$(MAKE) $(MAKESILENT) -f examples/centerofmass/CMakeFiles/centerofmass.dir/build.make examples/centerofmass/CMakeFiles/centerofmass.dir/build
.PHONY : centerofmass/fast

#=============================================================================
# Target rules for targets named collision

# Build rule for target.
collision: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 collision
.PHONY : collision

# fast build rule for target.
collision/fast:
	$(MAKE) $(MAKESILENT) -f examples/collision/CMakeFiles/collision.dir/build.make examples/collision/CMakeFiles/collision.dir/build
.PHONY : collision/fast

#=============================================================================
# Target rules for targets named dice

# Build rule for target.
dice: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 dice
.PHONY : dice

# fast build rule for target.
dice/fast:
	$(MAKE) $(MAKESILENT) -f examples/dice/CMakeFiles/dice.dir/build.make examples/dice/CMakeFiles/dice.dir/build
.PHONY : dice/fast

#=============================================================================
# Target rules for targets named hinge

# Build rule for target.
hinge: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 hinge
.PHONY : hinge

# fast build rule for target.
hinge/fast:
	$(MAKE) $(MAKESILENT) -f examples/hinge/CMakeFiles/hinge.dir/build.make examples/hinge/CMakeFiles/hinge.dir/build
.PHONY : hinge/fast

#=============================================================================
# Target rules for targets named multithreaded

# Build rule for target.
multithreaded: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 multithreaded
.PHONY : multithreaded

# fast build rule for target.
multithreaded/fast:
	$(MAKE) $(MAKESILENT) -f examples/multithreaded/CMakeFiles/multithreaded.dir/build.make examples/multithreaded/CMakeFiles/multithreaded.dir/build
.PHONY : multithreaded/fast

#=============================================================================
# Target rules for targets named patch-lowlevel

# Build rule for target.
patch-lowlevel: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 patch-lowlevel
.PHONY : patch-lowlevel

# fast build rule for target.
patch-lowlevel/fast:
	$(MAKE) $(MAKESILENT) -f examples/patch-lowlevel/CMakeFiles/patch-lowlevel.dir/build.make examples/patch-lowlevel/CMakeFiles/patch-lowlevel.dir/build
.PHONY : patch-lowlevel/fast

#=============================================================================
# Target rules for targets named slider

# Build rule for target.
slider: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 slider
.PHONY : slider

# fast build rule for target.
slider/fast:
	$(MAKE) $(MAKESILENT) -f examples/slider/CMakeFiles/slider.dir/build.make examples/slider/CMakeFiles/slider.dir/build
.PHONY : slider/fast

#=============================================================================
# Target rules for targets named saverestore

# Build rule for target.
saverestore: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 saverestore
.PHONY : saverestore

# fast build rule for target.
saverestore/fast:
	$(MAKE) $(MAKESILENT) -f examples/saverestore/CMakeFiles/saverestore.dir/build.make examples/saverestore/CMakeFiles/saverestore.dir/build
.PHONY : saverestore/fast

#=============================================================================
# Target rules for targets named osgbCollision

# Build rule for target.
osgbCollision: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 osgbCollision
.PHONY : osgbCollision

# fast build rule for target.
osgbCollision/fast:
	$(MAKE) $(MAKESILENT) -f src/osgbCollision/CMakeFiles/osgbCollision.dir/build.make src/osgbCollision/CMakeFiles/osgbCollision.dir/build
.PHONY : osgbCollision/fast

#=============================================================================
# Target rules for targets named osgbDynamics

# Build rule for target.
osgbDynamics: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 osgbDynamics
.PHONY : osgbDynamics

# fast build rule for target.
osgbDynamics/fast:
	$(MAKE) $(MAKESILENT) -f src/osgbDynamics/CMakeFiles/osgbDynamics.dir/build.make src/osgbDynamics/CMakeFiles/osgbDynamics.dir/build
.PHONY : osgbDynamics/fast

#=============================================================================
# Target rules for targets named osgbInteraction

# Build rule for target.
osgbInteraction: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 osgbInteraction
.PHONY : osgbInteraction

# fast build rule for target.
osgbInteraction/fast:
	$(MAKE) $(MAKESILENT) -f src/osgbInteraction/CMakeFiles/osgbInteraction.dir/build.make src/osgbInteraction/CMakeFiles/osgbInteraction.dir/build
.PHONY : osgbInteraction/fast

#=============================================================================
# Target rules for targets named osgdb_osgbdynamics

# Build rule for target.
osgdb_osgbdynamics: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 osgdb_osgbdynamics
.PHONY : osgdb_osgbdynamics

# fast build rule for target.
osgdb_osgbdynamics/fast:
	$(MAKE) $(MAKESILENT) -f src/plugins/osgdb_osgbDynamics/CMakeFiles/osgdb_osgbdynamics.dir/build.make src/plugins/osgdb_osgbDynamics/CMakeFiles/osgdb_osgbdynamics.dir/build
.PHONY : osgdb_osgbdynamics/fast

#=============================================================================
# Target rules for targets named osgdb_sgb

# Build rule for target.
osgdb_sgb: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 osgdb_sgb
.PHONY : osgdb_sgb

# fast build rule for target.
osgdb_sgb/fast:
	$(MAKE) $(MAKESILENT) -f src/plugins/osgdb_sgb/CMakeFiles/osgdb_sgb.dir/build.make src/plugins/osgdb_sgb/CMakeFiles/osgdb_sgb.dir/build
.PHONY : osgdb_sgb/fast

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... install"
	@echo "... install/local"
	@echo "... install/strip"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... BasicDemo"
	@echo "... centerofmass"
	@echo "... collision"
	@echo "... dice"
	@echo "... hinge"
	@echo "... multithreaded"
	@echo "... osgbCollision"
	@echo "... osgbDynamics"
	@echo "... osgbInteraction"
	@echo "... osgbpp"
	@echo "... osgdb_osgbdynamics"
	@echo "... osgdb_sgb"
	@echo "... patch-lowlevel"
	@echo "... saverestore"
	@echo "... slider"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

