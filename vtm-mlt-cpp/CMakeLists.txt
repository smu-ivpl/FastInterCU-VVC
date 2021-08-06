# minimum required cmake version
cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

# project name
if( EXTENSION_360_VIDEO )
  project( NextSoftware360 )
else()
  project( NextSoftware )
endif()

# use ccache
find_program( CCACHE_FOUND ccache )
if( CCACHE_FOUND )
  message( STATUS "ccache found. using it." )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_LINK ccache )
endif()

# set default CMAKE_BUILD_TYPE to Release if not set
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    set( USE_ADDRESS_SANITIZER OFF CACHE BOOL "Compiles with -sanitize=address and links to libasan" )
  endif()
endif()

set( EXTENSION_360_VIDEO OFF CACHE BOOL "If EXTENSION_360_VIDEO is on, 360Lib will be added" )
set( EXTENSION_HDRTOOLS OFF CACHE BOOL "If EXTENSION_HDRTOOLS is on, HDRLib will be added" )
set( SET_ENABLE_TRACING OFF CACHE BOOL "Set ENABLE_TRACING as a compiler flag" )
set( ENABLE_TRACING OFF CACHE BOOL "If SET_ENABLE_TRACING is on, it will be set to this value" )

if( CMAKE_COMPILER_IS_GNUCC )
  set( BUILD_STATIC OFF CACHE BOOL "Build static executables" )
endif()

# set c++11 -> 14
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# compile everything position independent (even static libraries)
set( CMAKE_POSITION_INDEPENDENT_CODE TRUE )

# set verbose compile options
#set( CMAKE_VERBOSE_MAKEFILE ON )

# use folders in IDEs for projects (e.g. lib sample app test)
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

# Include a utility module providing functions, macros, and settings
include( ${CMAKE_SOURCE_DIR}/cmake/CMakeBuild/cmake/modules/BBuildEnv.cmake )

# Enable multithreading
bb_multithreading()

# LibTorch
list(APPEND CMAKE_PREFIX_PATH "~/Libraries/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# OpenCV
set(OpenCV_DIR [OpenCV.cmake PATH])
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(OpenMP)

if( OpenMP_FOUND )
  set( CMAKE_C_FLAGS          "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
  set( CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
  set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}" )
  
  set( SET_ENABLE_SPLIT_PARALLELISM OFF CACHE BOOL "Set ENABLE_SPLIT_PARALLELISM as a compiler flag" )
  set( ENABLE_SPLIT_PARALLELISM     OFF CACHE BOOL "If SET_ENABLE_SPLIT_PARALLELISM is on, it will be set to this value" )
  set( SET_ENABLE_WPP_PARALLELISM   OFF CACHE BOOL "Set ENABLE_WPP_PARALLELISM as a compiler flag" )
  set( ENABLE_WPP_PARALLELISM       OFF CACHE BOOL "If SET_ENABLE_WPP_PARALLELISM is on, it will be set to this value" )
endif()

# Enable warnings for some generators and toolsets.
# bb_enable_warnings( gcc warnings-as-errors -Wno-sign-compare )
# bb_enable_warnings( gcc -Wno-unused-variable )
# bb_enable_warnings( gcc-4.8 warnings-as-errors -Wno-unused-variable )
# for gcc 8.2:
bb_enable_warnings( gcc warnings-as-errors -Wno-sign-compare -Wno-class-memaccess)

if( XCODE )
  bb_enable_warnings( clang warnings-as-errors
                            -Wno-deprecated-declarations
                            -Wno-unknown-attributes
                            -Wno-deprecated-register
                            -Wno-pessimizing-move
                            -Wno-absolute-value
                            -Wno-unused-const-variable )
else()
  bb_enable_warnings( clang warnings-as-errors
                            -Wno-unknown-attributes
                            -Wno-deprecated-register
                            -Wno-pessimizing-move
                            -Wno-absolute-value
                            -Wno-unused-const-variable )
endif()
#bb_enable_warnings( clang warnings-as-errors )

# enable warnings
bb_enable_warnings( msvc warnings-as-errors "/wd4996" )

# enable sse4.1 build for all source files for gcc and clang
if( UNIX OR MINGW )
  add_compile_options( "-msse4.1" )
endif()

# enable parallel build for Visual Studio
if( MSVC )
  add_compile_options( "/MP" )
  add_compile_options( "/EHsc" )
endif()

# set address sanitizer compiler arguments
if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    if( USE_ADDRESS_SANITIZER )
      # add compile options
      add_compile_options( "-fsanitize=address" )
      add_link_options( "-fsanitize=address" )
    endif()
  endif()

  if( CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0 )
    add_compile_options( "-fabi-version=6" )
  endif()
endif()

# modify .lldbinit for lldb custom data formatters
if( XCODE )
  set( LLDB_INSTALL_ROOT "$ENV{HOME}/.lldb.d" )
  set( LLDBINIT_FILE "$ENV{HOME}/.lldbinit" )
  set( ENABLE_LLDBINIT_UPDATE ON )
  # add custom target to install LLDB files.
  add_subdirectory( "lldb" )
endif()

# add needed subdirectories
add_subdirectory( "source/Lib/CommonLib" )
add_subdirectory( "source/Lib/CommonAnalyserLib" )
if( EXTENSION_360_VIDEO )
  add_subdirectory( "source/Lib/Lib360" )
  add_subdirectory( "source/Lib/AppEncHelper360" )
endif()
if ( EXTENSION_HDRTOOLS )
  add_subdirectory( "source/Lib/HDRLib")
endif()
add_subdirectory( "source/Lib/DecoderAnalyserLib" )
add_subdirectory( "source/Lib/DecoderLib" )
add_subdirectory( "source/Lib/EncoderLib" )
add_subdirectory( "source/Lib/Utilities" )

add_subdirectory( "source/App/DecoderAnalyserApp" )
add_subdirectory( "source/App/DecoderApp" )
add_subdirectory( "source/App/EncoderApp" )
add_subdirectory( "source/App/SEIRemovalApp" )
add_subdirectory( "source/App/Parcat" )
add_subdirectory( "source/App/StreamMergeApp" )
add_subdirectory( "source/App/BitstreamExtractorApp" )
add_subdirectory( "source/App/SubpicMergeApp" )
if( EXTENSION_360_VIDEO )
  add_subdirectory( "source/App/utils/360ConvertApp" )
endif()
