# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/peisen/TuningInterface

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peisen/TuningInterface/build

# Include any dependencies generated for this target.
include serialLib/CMakeFiles/serialLib.dir/depend.make

# Include the progress variables for this target.
include serialLib/CMakeFiles/serialLib.dir/progress.make

# Include the compile flags for this target's objects.
include serialLib/CMakeFiles/serialLib.dir/flags.make

serialLib/CMakeFiles/serialLib.dir/serialib.cpp.o: serialLib/CMakeFiles/serialLib.dir/flags.make
serialLib/CMakeFiles/serialLib.dir/serialib.cpp.o: ../serialLib/serialib.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peisen/TuningInterface/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serialLib/CMakeFiles/serialLib.dir/serialib.cpp.o"
	cd /home/peisen/TuningInterface/build/serialLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialLib.dir/serialib.cpp.o -c /home/peisen/TuningInterface/serialLib/serialib.cpp

serialLib/CMakeFiles/serialLib.dir/serialib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialLib.dir/serialib.cpp.i"
	cd /home/peisen/TuningInterface/build/serialLib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peisen/TuningInterface/serialLib/serialib.cpp > CMakeFiles/serialLib.dir/serialib.cpp.i

serialLib/CMakeFiles/serialLib.dir/serialib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialLib.dir/serialib.cpp.s"
	cd /home/peisen/TuningInterface/build/serialLib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peisen/TuningInterface/serialLib/serialib.cpp -o CMakeFiles/serialLib.dir/serialib.cpp.s

# Object files for target serialLib
serialLib_OBJECTS = \
"CMakeFiles/serialLib.dir/serialib.cpp.o"

# External object files for target serialLib
serialLib_EXTERNAL_OBJECTS =

serialLib/libserialLib.a: serialLib/CMakeFiles/serialLib.dir/serialib.cpp.o
serialLib/libserialLib.a: serialLib/CMakeFiles/serialLib.dir/build.make
serialLib/libserialLib.a: serialLib/CMakeFiles/serialLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peisen/TuningInterface/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libserialLib.a"
	cd /home/peisen/TuningInterface/build/serialLib && $(CMAKE_COMMAND) -P CMakeFiles/serialLib.dir/cmake_clean_target.cmake
	cd /home/peisen/TuningInterface/build/serialLib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serialLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serialLib/CMakeFiles/serialLib.dir/build: serialLib/libserialLib.a

.PHONY : serialLib/CMakeFiles/serialLib.dir/build

serialLib/CMakeFiles/serialLib.dir/clean:
	cd /home/peisen/TuningInterface/build/serialLib && $(CMAKE_COMMAND) -P CMakeFiles/serialLib.dir/cmake_clean.cmake
.PHONY : serialLib/CMakeFiles/serialLib.dir/clean

serialLib/CMakeFiles/serialLib.dir/depend:
	cd /home/peisen/TuningInterface/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peisen/TuningInterface /home/peisen/TuningInterface/serialLib /home/peisen/TuningInterface/build /home/peisen/TuningInterface/build/serialLib /home/peisen/TuningInterface/build/serialLib/CMakeFiles/serialLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serialLib/CMakeFiles/serialLib.dir/depend
