# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.6.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.6.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build

# Include any dependencies generated for this target.
include CMakeFiles/swarm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/swarm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/swarm.dir/flags.make

CMakeFiles/swarm.dir/src/swarm.cpp.o: CMakeFiles/swarm.dir/flags.make
CMakeFiles/swarm.dir/src/swarm.cpp.o: ../src/swarm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/swarm.dir/src/swarm.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/swarm.dir/src/swarm.cpp.o -c /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/swarm.cpp

CMakeFiles/swarm.dir/src/swarm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/swarm.dir/src/swarm.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/swarm.cpp > CMakeFiles/swarm.dir/src/swarm.cpp.i

CMakeFiles/swarm.dir/src/swarm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/swarm.dir/src/swarm.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/swarm.cpp -o CMakeFiles/swarm.dir/src/swarm.cpp.s

CMakeFiles/swarm.dir/src/swarm.cpp.o.requires:

.PHONY : CMakeFiles/swarm.dir/src/swarm.cpp.o.requires

CMakeFiles/swarm.dir/src/swarm.cpp.o.provides: CMakeFiles/swarm.dir/src/swarm.cpp.o.requires
	$(MAKE) -f CMakeFiles/swarm.dir/build.make CMakeFiles/swarm.dir/src/swarm.cpp.o.provides.build
.PHONY : CMakeFiles/swarm.dir/src/swarm.cpp.o.provides

CMakeFiles/swarm.dir/src/swarm.cpp.o.provides.build: CMakeFiles/swarm.dir/src/swarm.cpp.o


CMakeFiles/swarm.dir/src/rbe510.cpp.o: CMakeFiles/swarm.dir/flags.make
CMakeFiles/swarm.dir/src/rbe510.cpp.o: ../src/rbe510.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/swarm.dir/src/rbe510.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/swarm.dir/src/rbe510.cpp.o -c /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/rbe510.cpp

CMakeFiles/swarm.dir/src/rbe510.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/swarm.dir/src/rbe510.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/rbe510.cpp > CMakeFiles/swarm.dir/src/rbe510.cpp.i

CMakeFiles/swarm.dir/src/rbe510.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/swarm.dir/src/rbe510.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/rbe510.cpp -o CMakeFiles/swarm.dir/src/rbe510.cpp.s

CMakeFiles/swarm.dir/src/rbe510.cpp.o.requires:

.PHONY : CMakeFiles/swarm.dir/src/rbe510.cpp.o.requires

CMakeFiles/swarm.dir/src/rbe510.cpp.o.provides: CMakeFiles/swarm.dir/src/rbe510.cpp.o.requires
	$(MAKE) -f CMakeFiles/swarm.dir/build.make CMakeFiles/swarm.dir/src/rbe510.cpp.o.provides.build
.PHONY : CMakeFiles/swarm.dir/src/rbe510.cpp.o.provides

CMakeFiles/swarm.dir/src/rbe510.cpp.o.provides.build: CMakeFiles/swarm.dir/src/rbe510.cpp.o


CMakeFiles/swarm.dir/src/agent.cpp.o: CMakeFiles/swarm.dir/flags.make
CMakeFiles/swarm.dir/src/agent.cpp.o: ../src/agent.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/swarm.dir/src/agent.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/swarm.dir/src/agent.cpp.o -c /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/agent.cpp

CMakeFiles/swarm.dir/src/agent.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/swarm.dir/src/agent.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/agent.cpp > CMakeFiles/swarm.dir/src/agent.cpp.i

CMakeFiles/swarm.dir/src/agent.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/swarm.dir/src/agent.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/src/agent.cpp -o CMakeFiles/swarm.dir/src/agent.cpp.s

CMakeFiles/swarm.dir/src/agent.cpp.o.requires:

.PHONY : CMakeFiles/swarm.dir/src/agent.cpp.o.requires

CMakeFiles/swarm.dir/src/agent.cpp.o.provides: CMakeFiles/swarm.dir/src/agent.cpp.o.requires
	$(MAKE) -f CMakeFiles/swarm.dir/build.make CMakeFiles/swarm.dir/src/agent.cpp.o.provides.build
.PHONY : CMakeFiles/swarm.dir/src/agent.cpp.o.provides

CMakeFiles/swarm.dir/src/agent.cpp.o.provides.build: CMakeFiles/swarm.dir/src/agent.cpp.o


# Object files for target swarm
swarm_OBJECTS = \
"CMakeFiles/swarm.dir/src/swarm.cpp.o" \
"CMakeFiles/swarm.dir/src/rbe510.cpp.o" \
"CMakeFiles/swarm.dir/src/agent.cpp.o"

# External object files for target swarm
swarm_EXTERNAL_OBJECTS =

swarm: CMakeFiles/swarm.dir/src/swarm.cpp.o
swarm: CMakeFiles/swarm.dir/src/rbe510.cpp.o
swarm: CMakeFiles/swarm.dir/src/agent.cpp.o
swarm: CMakeFiles/swarm.dir/build.make
swarm: CMakeFiles/swarm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable swarm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/swarm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/swarm.dir/build: swarm

.PHONY : CMakeFiles/swarm.dir/build

CMakeFiles/swarm.dir/requires: CMakeFiles/swarm.dir/src/swarm.cpp.o.requires
CMakeFiles/swarm.dir/requires: CMakeFiles/swarm.dir/src/rbe510.cpp.o.requires
CMakeFiles/swarm.dir/requires: CMakeFiles/swarm.dir/src/agent.cpp.o.requires

.PHONY : CMakeFiles/swarm.dir/requires

CMakeFiles/swarm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/swarm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/swarm.dir/clean

CMakeFiles/swarm.dir/depend:
	cd /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build /Users/multirobot/Desktop/jgschornak/RBE510-SparkiSwarm/build/CMakeFiles/swarm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/swarm.dir/depend

