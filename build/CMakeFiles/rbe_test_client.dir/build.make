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
CMAKE_SOURCE_DIR = /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build

# Include any dependencies generated for this target.
include CMakeFiles/rbe_test_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rbe_test_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rbe_test_client.dir/flags.make

CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o: CMakeFiles/rbe_test_client.dir/flags.make
CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o: ../src/rbe_test_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o -c /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/src/rbe_test_client.cpp

CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/src/rbe_test_client.cpp > CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.i

CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/src/rbe_test_client.cpp -o CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.s

CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.requires:

.PHONY : CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.requires

CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.provides: CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.requires
	$(MAKE) -f CMakeFiles/rbe_test_client.dir/build.make CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.provides.build
.PHONY : CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.provides

CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.provides.build: CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o


CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o: CMakeFiles/rbe_test_client.dir/flags.make
CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o: ../src/rbe510.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o -c /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/src/rbe510.cpp

CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/src/rbe510.cpp > CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.i

CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/src/rbe510.cpp -o CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.s

CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.requires:

.PHONY : CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.requires

CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.provides: CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.requires
	$(MAKE) -f CMakeFiles/rbe_test_client.dir/build.make CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.provides.build
.PHONY : CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.provides

CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.provides.build: CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o


# Object files for target rbe_test_client
rbe_test_client_OBJECTS = \
"CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o" \
"CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o"

# External object files for target rbe_test_client
rbe_test_client_EXTERNAL_OBJECTS =

rbe_test_client: CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o
rbe_test_client: CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o
rbe_test_client: CMakeFiles/rbe_test_client.dir/build.make
rbe_test_client: CMakeFiles/rbe_test_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable rbe_test_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rbe_test_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rbe_test_client.dir/build: rbe_test_client

.PHONY : CMakeFiles/rbe_test_client.dir/build

CMakeFiles/rbe_test_client.dir/requires: CMakeFiles/rbe_test_client.dir/src/rbe_test_client.cpp.o.requires
CMakeFiles/rbe_test_client.dir/requires: CMakeFiles/rbe_test_client.dir/src/rbe510.cpp.o.requires

.PHONY : CMakeFiles/rbe_test_client.dir/requires

CMakeFiles/rbe_test_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rbe_test_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rbe_test_client.dir/clean

CMakeFiles/rbe_test_client.dir/depend:
	cd /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016 /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016 /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build /Users/multirobot/Desktop/WPI-RBE-MULTI-ROBOTICS-2016/build/CMakeFiles/rbe_test_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rbe_test_client.dir/depend

