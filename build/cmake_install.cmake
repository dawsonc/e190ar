<<<<<<< HEAD
# Install script for directory: /home/dawsonc/e190_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dawsonc/e190_ws/install")
=======
# Install script for directory: /home/peter/190_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/peter/190_ws/install")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/dawsonc/e190_ws/install/_setup_util.py")
=======
   "/home/peter/190_ws/install/_setup_util.py")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/dawsonc/e190_ws/install" TYPE PROGRAM FILES "/home/dawsonc/e190_ws/build/catkin_generated/installspace/_setup_util.py")
=======
file(INSTALL DESTINATION "/home/peter/190_ws/install" TYPE PROGRAM FILES "/home/peter/190_ws/build/catkin_generated/installspace/_setup_util.py")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/dawsonc/e190_ws/install/env.sh")
=======
   "/home/peter/190_ws/install/env.sh")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/dawsonc/e190_ws/install" TYPE PROGRAM FILES "/home/dawsonc/e190_ws/build/catkin_generated/installspace/env.sh")
=======
file(INSTALL DESTINATION "/home/peter/190_ws/install" TYPE PROGRAM FILES "/home/peter/190_ws/build/catkin_generated/installspace/env.sh")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/dawsonc/e190_ws/install/setup.bash")
=======
   "/home/peter/190_ws/install/setup.bash")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/dawsonc/e190_ws/install" TYPE FILE FILES "/home/dawsonc/e190_ws/build/catkin_generated/installspace/setup.bash")
=======
file(INSTALL DESTINATION "/home/peter/190_ws/install" TYPE FILE FILES "/home/peter/190_ws/build/catkin_generated/installspace/setup.bash")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/dawsonc/e190_ws/install/setup.sh")
=======
   "/home/peter/190_ws/install/setup.sh")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/dawsonc/e190_ws/install" TYPE FILE FILES "/home/dawsonc/e190_ws/build/catkin_generated/installspace/setup.sh")
=======
file(INSTALL DESTINATION "/home/peter/190_ws/install" TYPE FILE FILES "/home/peter/190_ws/build/catkin_generated/installspace/setup.sh")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/dawsonc/e190_ws/install/setup.zsh")
=======
   "/home/peter/190_ws/install/setup.zsh")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/dawsonc/e190_ws/install" TYPE FILE FILES "/home/dawsonc/e190_ws/build/catkin_generated/installspace/setup.zsh")
=======
file(INSTALL DESTINATION "/home/peter/190_ws/install" TYPE FILE FILES "/home/peter/190_ws/build/catkin_generated/installspace/setup.zsh")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/dawsonc/e190_ws/install/.rosinstall")
=======
   "/home/peter/190_ws/install/.rosinstall")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/dawsonc/e190_ws/install" TYPE FILE FILES "/home/dawsonc/e190_ws/build/catkin_generated/installspace/.rosinstall")
=======
file(INSTALL DESTINATION "/home/peter/190_ws/install" TYPE FILE FILES "/home/peter/190_ws/build/catkin_generated/installspace/.rosinstall")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
<<<<<<< HEAD
  include("/home/dawsonc/e190_ws/build/gtest/cmake_install.cmake")
  include("/home/dawsonc/e190_ws/build/e190_bot/cmake_install.cmake")
  include("/home/dawsonc/e190_ws/build/learning_tf/cmake_install.cmake")
=======
  include("/home/peter/190_ws/build/gtest/cmake_install.cmake")
  include("/home/peter/190_ws/build/e190_bot/cmake_install.cmake")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/dawsonc/e190_ws/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/peter/190_ws/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
