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
CMAKE_COMMAND = /home/robert/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/193.6015.37/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/robert/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/193.6015.37/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robert/sciebo/MaTIN/AMS/object_hunt_base

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/object_hunt_base.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/object_hunt_base.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/object_hunt_base.dir/flags.make

CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.o: CMakeFiles/object_hunt_base.dir/flags.make
CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.o: ../src/ObjectHunterBase_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/ObjectHunterBase_main.cpp

CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/ObjectHunterBase_main.cpp > CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.i

CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/ObjectHunterBase_main.cpp -o CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.s

CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.o: CMakeFiles/object_hunt_base.dir/flags.make
CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.o: ../src/TaskDistributor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/TaskDistributor.cpp

CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/TaskDistributor.cpp > CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.i

CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/TaskDistributor.cpp -o CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.s

CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.o: CMakeFiles/object_hunt_base.dir/flags.make
CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.o: ../src/HardwareInterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/HardwareInterface.cpp

CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/HardwareInterface.cpp > CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.i

CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/HardwareInterface.cpp -o CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.s

# Object files for target object_hunt_base
object_hunt_base_OBJECTS = \
"CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.o" \
"CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.o" \
"CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.o"

# External object files for target object_hunt_base
object_hunt_base_EXTERNAL_OBJECTS =

object_hunt_base: CMakeFiles/object_hunt_base.dir/src/ObjectHunterBase_main.cpp.o
object_hunt_base: CMakeFiles/object_hunt_base.dir/src/TaskDistributor.cpp.o
object_hunt_base: CMakeFiles/object_hunt_base.dir/src/HardwareInterface.cpp.o
object_hunt_base: CMakeFiles/object_hunt_base.dir/build.make
object_hunt_base: libsensors.a
object_hunt_base: libactuators.a
object_hunt_base: libfundamentals.a
object_hunt_base: CMakeFiles/object_hunt_base.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable object_hunt_base"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_hunt_base.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/object_hunt_base.dir/build: object_hunt_base

.PHONY : CMakeFiles/object_hunt_base.dir/build

CMakeFiles/object_hunt_base.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/object_hunt_base.dir/cmake_clean.cmake
.PHONY : CMakeFiles/object_hunt_base.dir/clean

CMakeFiles/object_hunt_base.dir/depend:
	cd /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robert/sciebo/MaTIN/AMS/object_hunt_base /home/robert/sciebo/MaTIN/AMS/object_hunt_base /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles/object_hunt_base.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/object_hunt_base.dir/depend

