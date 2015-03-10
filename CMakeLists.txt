cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(playgroung)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
include_directories(${PCL_INCLUDE_DIRS} include/dataFramework)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


SET(DATA_FRAMEWORK_SOURCES src/dataFramework/dataTransformations.cpp)
add_executable (lum src/lum.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (lum ${PCL_LIBRARIES})

add_executable (voxel_grid src/voxel_grid.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

add_executable (incrementalREG src/incrementalReg/incremental.cpp src/incrementalReg/viewerLog.cpp src/incrementalReg/viewerLog.hpp src/incrementalReg/configReader.cpp src/incrementalReg/configReader.hpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (incrementalREG ${PCL_LIBRARIES})

add_executable (testingPL  src/tests.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (testingPL ${PCL_LIBRARIES})


add_executable (datasetViewer  src/datasetViewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (datasetViewer ${PCL_LIBRARIES})

add_executable (converter  tool/fromFramesToPcd.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (converter ${PCL_LIBRARIES})

add_executable (trajectoryPlotter  tool/trajectoryPlotter.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (trajectoryPlotter ${PCL_LIBRARIES})

add_executable (fromSensorToPcd  tool/fromSensorToPcd.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (fromSensorToPcd ${PCL_LIBRARIES})

add_executable (elch src/elch.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (elch ${PCL_LIBRARIES})



