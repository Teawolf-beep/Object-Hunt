# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/pi/object_hunt/robert/object_hunt_base

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/object_hunt/robert/object_hunt_base/build

# Include any dependencies generated for this target.
include CMakeFiles/actuator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/actuator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/actuator.dir/flags.make

CMakeFiles/actuator.dir/src/stepper.cpp.o: CMakeFiles/actuator.dir/flags.make
CMakeFiles/actuator.dir/src/stepper.cpp.o: ../src/stepper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/object_hunt/robert/object_hunt_base/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/actuator.dir/src/stepper.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/actuator.dir/src/stepper.cpp.o -c /home/pi/object_hunt/robert/object_hunt_base/src/stepper.cpp

CMakeFiles/actuator.dir/src/stepper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/actuator.dir/src/stepper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/object_hunt/robert/object_hunt_base/src/stepper.cpp > CMakeFiles/actuator.dir/src/stepper.cpp.i

CMakeFiles/actuator.dir/src/stepper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/actuator.dir/src/stepper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/object_hunt/robert/object_hunt_base/src/stepper.cpp -o CMakeFiles/actuator.dir/src/stepper.cpp.s

CMakeFiles/actuator.dir/src/motor.cpp.o: CMakeFiles/actuator.dir/flags.make
CMakeFiles/actuator.dir/src/motor.cpp.o: ../src/motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/object_hunt/robert/object_hunt_base/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/actuator.dir/src/motor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/actuator.dir/src/motor.cpp.o -c /home/pi/object_hunt/robert/object_hunt_base/src/motor.cpp

CMakeFiles/actuator.dir/src/motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/actuator.dir/src/motor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/object_hunt/robert/object_hunt_base/src/motor.cpp > CMakeFiles/actuator.dir/src/motor.cpp.i

CMakeFiles/actuator.dir/src/motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/actuator.dir/src/motor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/object_hunt/robert/object_hunt_base/src/motor.cpp -o CMakeFiles/actuator.dir/src/motor.cpp.s

# Object files for target actuator
actuator_OBJECTS = \
"CMakeFiles/actuator.dir/src/stepper.cpp.o" \
"CMakeFiles/actuator.dir/src/motor.cpp.o"

# External object files for target actuator
actuator_EXTERNAL_OBJECTS =

libactuator.a: CMakeFiles/actuator.dir/src/stepper.cpp.o
libactuator.a: CMakeFiles/actuator.dir/src/motor.cpp.o
libactuator.a: CMakeFiles/actuator.dir/build.make
libactuator.a: CMakeFiles/actuator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/object_hunt/robert/object_hunt_base/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libactuator.a"
	$(CMAKE_COMMAND) -P CMakeFiles/actuator.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/actuator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/actuator.dir/build: libactuator.a

.PHONY : CMakeFiles/actuator.dir/build

CMakeFiles/actuator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/actuator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/actuator.dir/clean

CMakeFiles/actuator.dir/depend:
	cd /home/pi/object_hunt/robert/object_hunt_base/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/object_hunt/robert/object_hunt_base /home/pi/object_hunt/robert/object_hunt_base /home/pi/object_hunt/robert/object_hunt_base/build /home/pi/object_hunt/robert/object_hunt_base/build /home/pi/object_hunt/robert/object_hunt_base/build/CMakeFiles/actuator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/actuator.dir/depend
