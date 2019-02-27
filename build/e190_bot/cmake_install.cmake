<<<<<<< HEAD
# Install script for directory: /home/dawsonc/e190_ws/src/e190_bot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dawsonc/e190_ws/install")
=======
# Install script for directory: /home/peter/190_ws/src/e190_bot

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
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dawsonc/e190_ws/build/e190_bot/catkin_generated/installspace/e190_bot.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/e190_bot/msg" TYPE FILE FILES "/home/peter/190_ws/src/e190_bot/msg/ir_sensor.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/e190_bot/cmake" TYPE FILE FILES "/home/peter/190_ws/build/e190_bot/catkin_generated/installspace/e190_bot-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/peter/190_ws/devel/include/e190_bot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/peter/190_ws/devel/share/roseus/ros/e190_bot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/peter/190_ws/devel/share/common-lisp/ros/e190_bot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/peter/190_ws/devel/share/gennodejs/ros/e190_bot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/peter/190_ws/devel/lib/python2.7/dist-packages/e190_bot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/peter/190_ws/devel/lib/python2.7/dist-packages/e190_bot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/peter/190_ws/build/e190_bot/catkin_generated/installspace/e190_bot.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/e190_bot/cmake" TYPE FILE FILES "/home/peter/190_ws/build/e190_bot/catkin_generated/installspace/e190_bot-msg-extras.cmake")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/e190_bot/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/dawsonc/e190_ws/build/e190_bot/catkin_generated/installspace/e190_botConfig.cmake"
    "/home/dawsonc/e190_ws/build/e190_bot/catkin_generated/installspace/e190_botConfig-version.cmake"
=======
    "/home/peter/190_ws/build/e190_bot/catkin_generated/installspace/e190_botConfig.cmake"
    "/home/peter/190_ws/build/e190_bot/catkin_generated/installspace/e190_botConfig-version.cmake"
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/e190_bot" TYPE FILE FILES "/home/dawsonc/e190_ws/src/e190_bot/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/e190_bot" TYPE FILE FILES "/home/peter/190_ws/src/e190_bot/package.xml")
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
endif()

