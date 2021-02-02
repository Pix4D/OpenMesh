#!/bin/bash

COMPILER=$1
LANGUAGE=$2
BUILD_TYPE=$3
QTVERSION=$4
IWYU=$5

# Exit script on any error
set -e 

OPTIONS=""
MAKE_OPTIONS=""
BUILDPATH=""

# set GTEST path
OPTIONS="-DGTEST_ROOT=/usr/src/gtest/"

if [ "$COMPILER" == "gcc" ]; then
  echo "Building with GCC";
  BUILDPATH="gcc"

  # without icecc: no options required
  OPTIONS="$OPTIONS -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_C_COMPILER=/usr/bin/gcc"
  MAKE_OPTIONS="-j16"
  export ICECC_CXX=/usr/bin/g++ ; export ICECC_CC=/usr/bin/gcc

elif [ "$COMPILER" == "clang" ]; then

  OPTIONS="$OPTIONS -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang"
  echo "Building with CLANG";
  BUILDPATH="clang"  
fi  

if [ "$LANGUAGE" == "cpp98" ]; then
  echo "Building with C++98";
  BUILDPATH="$BUILDPATH-cpp98"
elif [ "$LANGUAGE" == "cpp11" ]; then
  echo "Building with C++11";
  OPTIONS="$OPTIONS -DCMAKE_CXX_FLAGS='-std=c++11' "
  BUILDPATH="$BUILDPATH-cpp11"  
elif [ "$LANGUAGE" == "cpp14" ]; then
  echo "Building with C++14";
  OPTIONS="$OPTIONS -DCMAKE_CXX_FLAGS='-std=c++14' "
  BUILDPATH="$BUILDPATH-cpp14"  
fi  

if [ "$QTVERSION" == "qt5.13.0" ]; then
  echo "Using QT5.13.0";
  BUILDPATH="$BUILDPATH-qt5.13.0"
  OPTIONS="$OPTIONS -DQWT6_INCLUDE_DIR=~/sw/qwt-6.1.4-qt5.13.0/include -DQWT6_LIBRARY_DIR=~/sw/qwt-6.1.4-qt5.13.0/lib -DQWT6_LIBRARY=~/sw/qwt-6.1.4-qt5.13.0/lib/libqwt.so -DQT5_INSTALL_PATH=~/sw/Qt/5.13.0/gcc_64"
elif [ "$QTVERSION" == "qt5.12.2" ]; then
  echo "Using QT5.12.2";
  BUILDPATH="$BUILDPATH-qt5.12.2"
  OPTIONS="$OPTIONS -DQWT6_INCLUDE_DIR=~/sw/qwt-6.1.4-qt5.12.2/include -DQWT6_LIBRARY_DIR=~/sw/qwt-6.1.4-qt5.12.2/lib -DQWT6_LIBRARY=~/sw/qwt-6.1.4-qt5.12.2/lib/libqwt.so -DQT5_INSTALL_PATH=~/sw/Qt/5.12.2/gcc_64"
elif [ "$QTVERSION" == "qt5.11.2" ]; then
  echo "Using QT5.11.2";
  BUILDPATH="$BUILDPATH-qt5.11.2"
  OPTIONS="$OPTIONS -DQWT6_INCLUDE_DIR=~/sw/qwt-6.1.3-qt5.11.2/include -DQWT6_LIBRARY_DIR=~/sw/qwt-6.1.3-qt5.11.2/lib -DQWT6_LIBRARY=~/sw/qwt-6.1.3-qt5.11.2/lib/libqwt.so -DQT5_INSTALL_PATH=~/sw/Qt/5.11.2/gcc_64"
elif [ "$QTVERSION" == "qt5.9.0" ]; then
  echo "Using QT5.9.0";
  BUILDPATH="$BUILDPATH-qt5.9.0"
  OPTIONS="$OPTIONS -DQWT6_INCLUDE_DIR=~/sw/qwt-6.1.3-qt5.9.0/include -DQWT6_LIBRARY_DIR=~/sw/qwt-6.1.3-qt5.9.0/lib -DQWT6_LIBRARY=~/sw/qwt-6.1.3-qt5.9.0/lib/libqwt.so -DQT5_INSTALL_PATH=~/sw/Qt/5.9/gcc_64"
elif [ "$QTVERSION" == "qt5.13.2" ]; then
  echo "Using QT5.13.2";
  BUILDPATH="$BUILDPATH-qt5.13.2"
  OPTIONS="$OPTIONS -DQWT6_INCLUDE_DIR=~/sw/qwt-6.1.4-qt5.13.2/include -DQWT6_LIBRARY_DIR=~/sw/qwt-6.1.4-qt5.13.2/lib -DQWT6_LIBRARY=~/sw/qwt-6.1.4-qt5.13.2/lib/libqwt.so -DQT5_INSTALL_PATH=~/sw/Qt/5.13.2/gcc_64"
elif [ "$QTVERSION" == "qt5.15.1" ]; then
  echo "Using QT5.15.1";
  BUILDPATH="$BUILDPATH-qt5.15.1"
  OPTIONS="$OPTIONS -DQWT6_INCLUDE_DIR=~/sw/qwt-6.1.5-qt5.15.1/include -DQWT6_LIBRARY_DIR=~/sw/qwt-6.1.5-qt5.15.1/lib -DQWT6_LIBRARY=~/sw/qwt-6.1.5-qt5.15.1/lib/libqwt.so -DQT5_INSTALL_PATH=~/sw/Qt/5.15.1/gcc_64"
fi

#=====================================
# Color Settings:
#=====================================
NC='\033[0m'
OUTPUT='\033[0;32m'
WARNING='\033[0;93m'

if [ "$BUILD_TYPE" == "release" ]; then
    export BUILD_TYPE=release
    BUILDPATH="$BUILDPATH-release"  
else
    export BUILD_TYPE=debug
    BUILDPATH="$BUILDPATH-debug"  
fi

if [ "$IWYU" == "IWYU" ]; then
  echo "Include what you use enabled.";
  BUILDPATH="$BUILDPATH-iwyu"
  OPTIONS="$OPTIONS -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
fi

echo "Building to directory $BUILDPATH"
