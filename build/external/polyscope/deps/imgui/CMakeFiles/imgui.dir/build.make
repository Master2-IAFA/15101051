# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/olivierd/Documents/15101051

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/olivierd/Documents/15101051/build

# Include any dependencies generated for this target.
include external/polyscope/deps/imgui/CMakeFiles/imgui.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.make

# Include the progress variables for this target.
include external/polyscope/deps/imgui/CMakeFiles/imgui.dir/progress.make

# Include the compile flags for this target's objects.
include external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o: ../external/polyscope/deps/imgui/imgui/imgui.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui.cpp > CMakeFiles/imgui.dir/imgui/imgui.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui.cpp -o CMakeFiles/imgui.dir/imgui/imgui.cpp.s

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o: ../external/polyscope/deps/imgui/imgui/imgui_draw.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_draw.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_draw.cpp > CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_draw.cpp -o CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.s

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o: ../external/polyscope/deps/imgui/imgui/imgui_tables.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_tables.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_tables.cpp > CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_tables.cpp -o CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.s

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o: ../external/polyscope/deps/imgui/imgui/imgui_widgets.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_widgets.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_widgets.cpp > CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_widgets.cpp -o CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.s

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o: ../external/polyscope/deps/imgui/imgui/imgui_demo.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o -MF CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o.d -o CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_demo.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_demo.cpp > CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/imgui_demo.cpp -o CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.s

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o: ../external/polyscope/deps/imgui/imgui/backends/imgui_impl_glfw.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o -MF CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o.d -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/backends/imgui_impl_glfw.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/backends/imgui_impl_glfw.cpp > CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/backends/imgui_impl_glfw.cpp -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.s

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/flags.make
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o: ../external/polyscope/deps/imgui/imgui/backends/imgui_impl_opengl3.cpp
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o -MF CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o.d -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o -c /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/backends/imgui_impl_opengl3.cpp

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.i"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/backends/imgui_impl_opengl3.cpp > CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.i

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.s"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/olivierd/Documents/15101051/external/polyscope/deps/imgui/imgui/backends/imgui_impl_opengl3.cpp -o CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.s

# Object files for target imgui
imgui_OBJECTS = \
"CMakeFiles/imgui.dir/imgui/imgui.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o" \
"CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o" \
"CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o" \
"CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o"

# External object files for target imgui
imgui_EXTERNAL_OBJECTS =

external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_draw.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_tables.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_widgets.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/imgui_demo.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_glfw.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/imgui/backends/imgui_impl_opengl3.cpp.o
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/build.make
external/polyscope/deps/imgui/libimgui.a: external/polyscope/deps/imgui/CMakeFiles/imgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/olivierd/Documents/15101051/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libimgui.a"
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && $(CMAKE_COMMAND) -P CMakeFiles/imgui.dir/cmake_clean_target.cmake
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imgui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
external/polyscope/deps/imgui/CMakeFiles/imgui.dir/build: external/polyscope/deps/imgui/libimgui.a
.PHONY : external/polyscope/deps/imgui/CMakeFiles/imgui.dir/build

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/clean:
	cd /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui && $(CMAKE_COMMAND) -P CMakeFiles/imgui.dir/cmake_clean.cmake
.PHONY : external/polyscope/deps/imgui/CMakeFiles/imgui.dir/clean

external/polyscope/deps/imgui/CMakeFiles/imgui.dir/depend:
	cd /home/olivierd/Documents/15101051/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/olivierd/Documents/15101051 /home/olivierd/Documents/15101051/external/polyscope/deps/imgui /home/olivierd/Documents/15101051/build /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui /home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui/CMakeFiles/imgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : external/polyscope/deps/imgui/CMakeFiles/imgui.dir/depend
