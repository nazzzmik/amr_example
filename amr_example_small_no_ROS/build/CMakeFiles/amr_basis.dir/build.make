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
CMAKE_SOURCE_DIR = /code/amr_ex_wts

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /code/amr_ex_wts/build

# Include any dependencies generated for this target.
include CMakeFiles/amr_basis.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/amr_basis.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/amr_basis.dir/flags.make

CMakeFiles/amr_basis.dir/src/amr_interface.cpp.o: CMakeFiles/amr_basis.dir/flags.make
CMakeFiles/amr_basis.dir/src/amr_interface.cpp.o: ../src/amr_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/amr_ex_wts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/amr_basis.dir/src/amr_interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amr_basis.dir/src/amr_interface.cpp.o -c /code/amr_ex_wts/src/amr_interface.cpp

CMakeFiles/amr_basis.dir/src/amr_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amr_basis.dir/src/amr_interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /code/amr_ex_wts/src/amr_interface.cpp > CMakeFiles/amr_basis.dir/src/amr_interface.cpp.i

CMakeFiles/amr_basis.dir/src/amr_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amr_basis.dir/src/amr_interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /code/amr_ex_wts/src/amr_interface.cpp -o CMakeFiles/amr_basis.dir/src/amr_interface.cpp.s

CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.o: CMakeFiles/amr_basis.dir/flags.make
CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.o: ../src/amr_task_executors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/amr_ex_wts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.o -c /code/amr_ex_wts/src/amr_task_executors.cpp

CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /code/amr_ex_wts/src/amr_task_executors.cpp > CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.i

CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /code/amr_ex_wts/src/amr_task_executors.cpp -o CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.s

CMakeFiles/amr_basis.dir/src/amr_unit.cpp.o: CMakeFiles/amr_basis.dir/flags.make
CMakeFiles/amr_basis.dir/src/amr_unit.cpp.o: ../src/amr_unit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/amr_ex_wts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/amr_basis.dir/src/amr_unit.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amr_basis.dir/src/amr_unit.cpp.o -c /code/amr_ex_wts/src/amr_unit.cpp

CMakeFiles/amr_basis.dir/src/amr_unit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amr_basis.dir/src/amr_unit.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /code/amr_ex_wts/src/amr_unit.cpp > CMakeFiles/amr_basis.dir/src/amr_unit.cpp.i

CMakeFiles/amr_basis.dir/src/amr_unit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amr_basis.dir/src/amr_unit.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /code/amr_ex_wts/src/amr_unit.cpp -o CMakeFiles/amr_basis.dir/src/amr_unit.cpp.s

CMakeFiles/amr_basis.dir/src/basic_routines.cpp.o: CMakeFiles/amr_basis.dir/flags.make
CMakeFiles/amr_basis.dir/src/basic_routines.cpp.o: ../src/basic_routines.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/amr_ex_wts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/amr_basis.dir/src/basic_routines.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amr_basis.dir/src/basic_routines.cpp.o -c /code/amr_ex_wts/src/basic_routines.cpp

CMakeFiles/amr_basis.dir/src/basic_routines.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amr_basis.dir/src/basic_routines.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /code/amr_ex_wts/src/basic_routines.cpp > CMakeFiles/amr_basis.dir/src/basic_routines.cpp.i

CMakeFiles/amr_basis.dir/src/basic_routines.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amr_basis.dir/src/basic_routines.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /code/amr_ex_wts/src/basic_routines.cpp -o CMakeFiles/amr_basis.dir/src/basic_routines.cpp.s

# Object files for target amr_basis
amr_basis_OBJECTS = \
"CMakeFiles/amr_basis.dir/src/amr_interface.cpp.o" \
"CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.o" \
"CMakeFiles/amr_basis.dir/src/amr_unit.cpp.o" \
"CMakeFiles/amr_basis.dir/src/basic_routines.cpp.o"

# External object files for target amr_basis
amr_basis_EXTERNAL_OBJECTS =

libamr_basis.a: CMakeFiles/amr_basis.dir/src/amr_interface.cpp.o
libamr_basis.a: CMakeFiles/amr_basis.dir/src/amr_task_executors.cpp.o
libamr_basis.a: CMakeFiles/amr_basis.dir/src/amr_unit.cpp.o
libamr_basis.a: CMakeFiles/amr_basis.dir/src/basic_routines.cpp.o
libamr_basis.a: CMakeFiles/amr_basis.dir/build.make
libamr_basis.a: CMakeFiles/amr_basis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/code/amr_ex_wts/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libamr_basis.a"
	$(CMAKE_COMMAND) -P CMakeFiles/amr_basis.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/amr_basis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/amr_basis.dir/build: libamr_basis.a

.PHONY : CMakeFiles/amr_basis.dir/build

CMakeFiles/amr_basis.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/amr_basis.dir/cmake_clean.cmake
.PHONY : CMakeFiles/amr_basis.dir/clean

CMakeFiles/amr_basis.dir/depend:
	cd /code/amr_ex_wts/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /code/amr_ex_wts /code/amr_ex_wts /code/amr_ex_wts/build /code/amr_ex_wts/build /code/amr_ex_wts/build/CMakeFiles/amr_basis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/amr_basis.dir/depend

