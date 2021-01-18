#Besm ALLAH
#sudo apt-get install libopenscenegraph-dev openscenegraph, remove before osgearth
#sudo apt-get install osgearth libosgearth-dev, also installs openscenegraph correct version
#sudo apt-get install libopencv-dev
#ffmpeg:
#sudo add-apt-repository ppa:mc3man/trusty-media  
#sudo apt-get update  
#sudo apt-get install ffmpeg  
#sudo apt-get install frei0r-plugins
#
#sudo apt-get install libboost-dev
#
#check package locations:
#check by dpkg -L libopenscenegraph-dev
#or find sth by: find / -name "avcodec.h"
#or see installed packages by: dpkg --get-selections
cmake_minimum_required(VERSION 2.8)
project( opencvCalib)

#Libraries#################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./") #also search current directory for Find*.cmake modules
##OpenCV################
find_package( OpenCV REQUIRED )

if(OpenCV_FOUND)
	message("Found Opencv!")
else()
	message("Can't find Opencv!")
endif()

include_directories(
    "${OpenCV_INCLUDE_DIRS}"
    "${PROJECT_SOURCE_DIR}/"
)

set( mSOURCES 
	camera_calibration.cpp
	Settings.cpp)

add_executable( opencvCalib ${mSOURCES})

target_link_libraries( opencvCalib ${OpenCV_LIBS} )



