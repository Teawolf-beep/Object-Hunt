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
include CMakeFiles/fundamentals.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fundamentals.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fundamentals.dir/flags.make

CMakeFiles/fundamentals.dir/src/GPIO.cpp.o: CMakeFiles/fundamentals.dir/flags.make
CMakeFiles/fundamentals.dir/src/GPIO.cpp.o: ../src/GPIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fundamentals.dir/src/GPIO.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fundamentals.dir/src/GPIO.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/GPIO.cpp

CMakeFiles/fundamentals.dir/src/GPIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fundamentals.dir/src/GPIO.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/GPIO.cpp > CMakeFiles/fundamentals.dir/src/GPIO.cpp.i

CMakeFiles/fundamentals.dir/src/GPIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fundamentals.dir/src/GPIO.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/GPIO.cpp -o CMakeFiles/fundamentals.dir/src/GPIO.cpp.s

CMakeFiles/fundamentals.dir/src/I2C.cpp.o: CMakeFiles/fundamentals.dir/flags.make
CMakeFiles/fundamentals.dir/src/I2C.cpp.o: ../src/I2C.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/fundamentals.dir/src/I2C.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fundamentals.dir/src/I2C.cpp.o -c /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/I2C.cpp

CMakeFiles/fundamentals.dir/src/I2C.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fundamentals.dir/src/I2C.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/I2C.cpp > CMakeFiles/fundamentals.dir/src/I2C.cpp.i

CMakeFiles/fundamentals.dir/src/I2C.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fundamentals.dir/src/I2C.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robert/sciebo/MaTIN/AMS/object_hunt_base/src/I2C.cpp -o CMakeFiles/fundamentals.dir/src/I2C.cpp.s

# Object files for target fundamentals
fundamentals_OBJECTS = \
"CMakeFiles/fundamentals.dir/src/GPIO.cpp.o" \
"CMakeFiles/fundamentals.dir/src/I2C.cpp.o"

# External object files for target fundamentals
fundamentals_EXTERNAL_OBJECTS =

libfundamentals.a: CMakeFiles/fundamentals.dir/src/GPIO.cpp.o
libfundamentals.a: CMakeFiles/fundamentals.dir/src/I2C.cpp.o
libfundamentals.a: CMakeFiles/fundamentals.dir/build.make
libfundamentals.a: CMakeFiles/fundamentals.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libfundamentals.a"
	$(CMAKE_COMMAND) -P CMakeFiles/fundamentals.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fundamentals.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fundamentals.dir/build: libfundamentals.a

.PHONY : CMakeFiles/fundamentals.dir/build

CMakeFiles/fundamentals.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fundamentals.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fundamentals.dir/clean

CMakeFiles/fundamentals.dir/depend:
	cd /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robert/sciebo/MaTIN/AMS/object_hunt_base /home/robert/sciebo/MaTIN/AMS/object_hunt_base /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug /home/robert/sciebo/MaTIN/AMS/object_hunt_base/cmake-build-debug/CMakeFiles/fundamentals.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fundamentals.dir/depend

