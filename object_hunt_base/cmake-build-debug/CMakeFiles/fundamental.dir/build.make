# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/robert/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/robert/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robert/sciebo/MaTIN/AMS/object_hunt_base

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/fundamental.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fundamental.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fundamental.dir/flags.make

CMakeFiles/fundamental.dir/src/gpio.cpp.o: CMakeFiles/fundamental.dir/flags.make
CMakeFiles/fundamental.dir/src/gpio.cpp.o: ../src/gpio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fundamental.dir/src/gpio.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fundamental.dir/src/gpio.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/gpio.cpp

CMakeFiles/fundamental.dir/src/gpio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fundamental.dir/src/gpio.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/gpio.cpp > CMakeFiles/fundamental.dir/src/gpio.cpp.i

CMakeFiles/fundamental.dir/src/gpio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fundamental.dir/src/gpio.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/gpio.cpp -o CMakeFiles/fundamental.dir/src/gpio.cpp.s

CMakeFiles/fundamental.dir/src/i2c.cpp.o: CMakeFiles/fundamental.dir/flags.make
CMakeFiles/fundamental.dir/src/i2c.cpp.o: ../src/i2c.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/fundamental.dir/src/i2c.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fundamental.dir/src/i2c.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/i2c.cpp

CMakeFiles/fundamental.dir/src/i2c.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fundamental.dir/src/i2c.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/i2c.cpp > CMakeFiles/fundamental.dir/src/i2c.cpp.i

CMakeFiles/fundamental.dir/src/i2c.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fundamental.dir/src/i2c.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/i2c.cpp -o CMakeFiles/fundamental.dir/src/i2c.cpp.s

# Object files for target fundamental
fundamental_OBJECTS = \
"CMakeFiles/fundamental.dir/src/gpio.cpp.o" \
"CMakeFiles/fundamental.dir/src/i2c.cpp.o"

# External object files for target fundamental
fundamental_EXTERNAL_OBJECTS =

libfundamental.a: CMakeFiles/fundamental.dir/src/gpio.cpp.o
libfundamental.a: CMakeFiles/fundamental.dir/src/i2c.cpp.o
libfundamental.a: CMakeFiles/fundamental.dir/build.make
libfundamental.a: CMakeFiles/fundamental.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libfundamental.a"
	$(CMAKE_COMMAND) -P CMakeFiles/fundamental.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fundamental.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fundamental.dir/build: libfundamental.a

.PHONY : CMakeFiles/fundamental.dir/build

CMakeFiles/fundamental.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fundamental.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fundamental.dir/clean

CMakeFiles/fundamental.dir/depend:
	cd /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robert/sciebo/MaTIN/AMS/object_hunt_base /home/robert/sciebo/MaTIN/AMS/object_hunt_base /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles/fundamental.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fundamental.dir/depend

