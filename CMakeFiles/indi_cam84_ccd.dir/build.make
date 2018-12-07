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
CMAKE_SOURCE_DIR = /home/michael/Desktop/cam84_development/cam84_updates

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/michael/Desktop/cam84_development/cam84_updates

# Include any dependencies generated for this target.
include CMakeFiles/indi_cam84_ccd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/indi_cam84_ccd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/indi_cam84_ccd.dir/flags.make

CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o: CMakeFiles/indi_cam84_ccd.dir/flags.make
CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o: cam84_ccd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michael/Desktop/cam84_development/cam84_updates/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o -c /home/michael/Desktop/cam84_development/cam84_updates/cam84_ccd.cpp

CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michael/Desktop/cam84_development/cam84_updates/cam84_ccd.cpp > CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.i

CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michael/Desktop/cam84_development/cam84_updates/cam84_ccd.cpp -o CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.s

CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.requires:

.PHONY : CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.requires

CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.provides: CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.requires
	$(MAKE) -f CMakeFiles/indi_cam84_ccd.dir/build.make CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.provides.build
.PHONY : CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.provides

CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.provides.build: CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o


# Object files for target indi_cam84_ccd
indi_cam84_ccd_OBJECTS = \
"CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o"

# External object files for target indi_cam84_ccd
indi_cam84_ccd_EXTERNAL_OBJECTS =

indi_cam84_ccd: CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o
indi_cam84_ccd: CMakeFiles/indi_cam84_ccd.dir/build.make
indi_cam84_ccd: libcam84.a
indi_cam84_ccd: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
indi_cam84_ccd: /usr/lib/x86_64-linux-gnu/libindidriver.so
indi_cam84_ccd: /usr/lib/x86_64-linux-gnu/libindiAlignmentDriver.so
indi_cam84_ccd: /usr/lib/x86_64-linux-gnu/libcfitsio.so
indi_cam84_ccd: /usr/lib/x86_64-linux-gnu/libz.so
indi_cam84_ccd: CMakeFiles/indi_cam84_ccd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/michael/Desktop/cam84_development/cam84_updates/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable indi_cam84_ccd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/indi_cam84_ccd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/indi_cam84_ccd.dir/build: indi_cam84_ccd

.PHONY : CMakeFiles/indi_cam84_ccd.dir/build

CMakeFiles/indi_cam84_ccd.dir/requires: CMakeFiles/indi_cam84_ccd.dir/cam84_ccd.o.requires

.PHONY : CMakeFiles/indi_cam84_ccd.dir/requires

CMakeFiles/indi_cam84_ccd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/indi_cam84_ccd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/indi_cam84_ccd.dir/clean

CMakeFiles/indi_cam84_ccd.dir/depend:
	cd /home/michael/Desktop/cam84_development/cam84_updates && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/michael/Desktop/cam84_development/cam84_updates /home/michael/Desktop/cam84_development/cam84_updates /home/michael/Desktop/cam84_development/cam84_updates /home/michael/Desktop/cam84_development/cam84_updates /home/michael/Desktop/cam84_development/cam84_updates/CMakeFiles/indi_cam84_ccd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/indi_cam84_ccd.dir/depend

