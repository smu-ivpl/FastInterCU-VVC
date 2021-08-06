# Install script for directory: /home/ubuntu/whyeo/vtm-mlt-final

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ubuntu/whyeo/vtm-mlt-final/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/Lib/CommonLib/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/Lib/CommonAnalyserLib/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/Lib/DecoderAnalyserLib/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/Lib/DecoderLib/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/Lib/EncoderLib/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/Lib/Utilities/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/DecoderAnalyserApp/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/DecoderApp/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/EncoderApp/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/SEIRemovalApp/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/Parcat/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/StreamMergeApp/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/BitstreamExtractorApp/cmake_install.cmake")
  include("/home/ubuntu/whyeo/vtm-mlt-final/build/source/App/SubpicMergeApp/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ubuntu/whyeo/vtm-mlt-final/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
