<<<<<<< Updated upstream
# Install script for directory: /home/olivierd/Documents/15101051/external/polyscope/deps
=======
# Install script for directory: /home/antoine/Desktop/15101051/external/polyscope/deps
>>>>>>> Stashed changes

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
<<<<<<< Updated upstream
    set(CMAKE_INSTALL_CONFIG_NAME "")
=======
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
>>>>>>> Stashed changes
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< Updated upstream
  include("/home/olivierd/Documents/15101051/build/external/polyscope/deps/glad/cmake_install.cmake")
=======
  include("/home/antoine/Desktop/15101051/build/external/polyscope/deps/glad/cmake_install.cmake")
>>>>>>> Stashed changes
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< Updated upstream
  include("/home/olivierd/Documents/15101051/build/external/polyscope/deps/glfw/cmake_install.cmake")
=======
  include("/home/antoine/Desktop/15101051/build/external/polyscope/deps/glfw/cmake_install.cmake")
>>>>>>> Stashed changes
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< Updated upstream
  include("/home/olivierd/Documents/15101051/build/external/polyscope/deps/imgui/cmake_install.cmake")
=======
  include("/home/antoine/Desktop/15101051/build/external/polyscope/deps/imgui/cmake_install.cmake")
>>>>>>> Stashed changes
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< Updated upstream
  include("/home/olivierd/Documents/15101051/build/external/polyscope/deps/stb/cmake_install.cmake")
=======
  include("/home/antoine/Desktop/15101051/build/external/polyscope/deps/stb/cmake_install.cmake")
>>>>>>> Stashed changes
endif()

