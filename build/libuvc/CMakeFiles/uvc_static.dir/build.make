# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

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
CMAKE_SOURCE_DIR = /home/tractor/ws/src/libuvc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tractor/ws/build/libuvc

# Include any dependencies generated for this target.
include CMakeFiles/uvc_static.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/uvc_static.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/uvc_static.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/uvc_static.dir/flags.make

CMakeFiles/uvc_static.dir/src/ctrl.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/ctrl.c.o: /home/tractor/ws/src/libuvc/src/ctrl.c
CMakeFiles/uvc_static.dir/src/ctrl.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/uvc_static.dir/src/ctrl.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/ctrl.c.o -MF CMakeFiles/uvc_static.dir/src/ctrl.c.o.d -o CMakeFiles/uvc_static.dir/src/ctrl.c.o -c /home/tractor/ws/src/libuvc/src/ctrl.c

CMakeFiles/uvc_static.dir/src/ctrl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/ctrl.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/ctrl.c > CMakeFiles/uvc_static.dir/src/ctrl.c.i

CMakeFiles/uvc_static.dir/src/ctrl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/ctrl.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/ctrl.c -o CMakeFiles/uvc_static.dir/src/ctrl.c.s

CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o: /home/tractor/ws/src/libuvc/src/ctrl-gen.c
CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o -MF CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o.d -o CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o -c /home/tractor/ws/src/libuvc/src/ctrl-gen.c

CMakeFiles/uvc_static.dir/src/ctrl-gen.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/ctrl-gen.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/ctrl-gen.c > CMakeFiles/uvc_static.dir/src/ctrl-gen.c.i

CMakeFiles/uvc_static.dir/src/ctrl-gen.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/ctrl-gen.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/ctrl-gen.c -o CMakeFiles/uvc_static.dir/src/ctrl-gen.c.s

CMakeFiles/uvc_static.dir/src/device.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/device.c.o: /home/tractor/ws/src/libuvc/src/device.c
CMakeFiles/uvc_static.dir/src/device.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/uvc_static.dir/src/device.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/device.c.o -MF CMakeFiles/uvc_static.dir/src/device.c.o.d -o CMakeFiles/uvc_static.dir/src/device.c.o -c /home/tractor/ws/src/libuvc/src/device.c

CMakeFiles/uvc_static.dir/src/device.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/device.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/device.c > CMakeFiles/uvc_static.dir/src/device.c.i

CMakeFiles/uvc_static.dir/src/device.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/device.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/device.c -o CMakeFiles/uvc_static.dir/src/device.c.s

CMakeFiles/uvc_static.dir/src/diag.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/diag.c.o: /home/tractor/ws/src/libuvc/src/diag.c
CMakeFiles/uvc_static.dir/src/diag.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/uvc_static.dir/src/diag.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/diag.c.o -MF CMakeFiles/uvc_static.dir/src/diag.c.o.d -o CMakeFiles/uvc_static.dir/src/diag.c.o -c /home/tractor/ws/src/libuvc/src/diag.c

CMakeFiles/uvc_static.dir/src/diag.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/diag.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/diag.c > CMakeFiles/uvc_static.dir/src/diag.c.i

CMakeFiles/uvc_static.dir/src/diag.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/diag.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/diag.c -o CMakeFiles/uvc_static.dir/src/diag.c.s

CMakeFiles/uvc_static.dir/src/frame.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/frame.c.o: /home/tractor/ws/src/libuvc/src/frame.c
CMakeFiles/uvc_static.dir/src/frame.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/uvc_static.dir/src/frame.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/frame.c.o -MF CMakeFiles/uvc_static.dir/src/frame.c.o.d -o CMakeFiles/uvc_static.dir/src/frame.c.o -c /home/tractor/ws/src/libuvc/src/frame.c

CMakeFiles/uvc_static.dir/src/frame.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/frame.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/frame.c > CMakeFiles/uvc_static.dir/src/frame.c.i

CMakeFiles/uvc_static.dir/src/frame.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/frame.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/frame.c -o CMakeFiles/uvc_static.dir/src/frame.c.s

CMakeFiles/uvc_static.dir/src/init.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/init.c.o: /home/tractor/ws/src/libuvc/src/init.c
CMakeFiles/uvc_static.dir/src/init.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/uvc_static.dir/src/init.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/init.c.o -MF CMakeFiles/uvc_static.dir/src/init.c.o.d -o CMakeFiles/uvc_static.dir/src/init.c.o -c /home/tractor/ws/src/libuvc/src/init.c

CMakeFiles/uvc_static.dir/src/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/init.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/init.c > CMakeFiles/uvc_static.dir/src/init.c.i

CMakeFiles/uvc_static.dir/src/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/init.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/init.c -o CMakeFiles/uvc_static.dir/src/init.c.s

CMakeFiles/uvc_static.dir/src/stream.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/stream.c.o: /home/tractor/ws/src/libuvc/src/stream.c
CMakeFiles/uvc_static.dir/src/stream.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/uvc_static.dir/src/stream.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/stream.c.o -MF CMakeFiles/uvc_static.dir/src/stream.c.o.d -o CMakeFiles/uvc_static.dir/src/stream.c.o -c /home/tractor/ws/src/libuvc/src/stream.c

CMakeFiles/uvc_static.dir/src/stream.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/stream.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/stream.c > CMakeFiles/uvc_static.dir/src/stream.c.i

CMakeFiles/uvc_static.dir/src/stream.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/stream.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/stream.c -o CMakeFiles/uvc_static.dir/src/stream.c.s

CMakeFiles/uvc_static.dir/src/misc.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/misc.c.o: /home/tractor/ws/src/libuvc/src/misc.c
CMakeFiles/uvc_static.dir/src/misc.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/uvc_static.dir/src/misc.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/misc.c.o -MF CMakeFiles/uvc_static.dir/src/misc.c.o.d -o CMakeFiles/uvc_static.dir/src/misc.c.o -c /home/tractor/ws/src/libuvc/src/misc.c

CMakeFiles/uvc_static.dir/src/misc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/misc.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/misc.c > CMakeFiles/uvc_static.dir/src/misc.c.i

CMakeFiles/uvc_static.dir/src/misc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/misc.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/misc.c -o CMakeFiles/uvc_static.dir/src/misc.c.s

CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o: CMakeFiles/uvc_static.dir/flags.make
CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o: /home/tractor/ws/src/libuvc/src/frame-mjpeg.c
CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o: CMakeFiles/uvc_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o -MF CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o.d -o CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o -c /home/tractor/ws/src/libuvc/src/frame-mjpeg.c

CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tractor/ws/src/libuvc/src/frame-mjpeg.c > CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.i

CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tractor/ws/src/libuvc/src/frame-mjpeg.c -o CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.s

# Object files for target uvc_static
uvc_static_OBJECTS = \
"CMakeFiles/uvc_static.dir/src/ctrl.c.o" \
"CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o" \
"CMakeFiles/uvc_static.dir/src/device.c.o" \
"CMakeFiles/uvc_static.dir/src/diag.c.o" \
"CMakeFiles/uvc_static.dir/src/frame.c.o" \
"CMakeFiles/uvc_static.dir/src/init.c.o" \
"CMakeFiles/uvc_static.dir/src/stream.c.o" \
"CMakeFiles/uvc_static.dir/src/misc.c.o" \
"CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o"

# External object files for target uvc_static
uvc_static_EXTERNAL_OBJECTS =

libuvc.a: CMakeFiles/uvc_static.dir/src/ctrl.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/ctrl-gen.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/device.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/diag.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/frame.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/init.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/stream.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/misc.c.o
libuvc.a: CMakeFiles/uvc_static.dir/src/frame-mjpeg.c.o
libuvc.a: CMakeFiles/uvc_static.dir/build.make
libuvc.a: CMakeFiles/uvc_static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tractor/ws/build/libuvc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking C static library libuvc.a"
	$(CMAKE_COMMAND) -P CMakeFiles/uvc_static.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uvc_static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/uvc_static.dir/build: libuvc.a
.PHONY : CMakeFiles/uvc_static.dir/build

CMakeFiles/uvc_static.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/uvc_static.dir/cmake_clean.cmake
.PHONY : CMakeFiles/uvc_static.dir/clean

CMakeFiles/uvc_static.dir/depend:
	cd /home/tractor/ws/build/libuvc && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tractor/ws/src/libuvc /home/tractor/ws/src/libuvc /home/tractor/ws/build/libuvc /home/tractor/ws/build/libuvc /home/tractor/ws/build/libuvc/CMakeFiles/uvc_static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/uvc_static.dir/depend

