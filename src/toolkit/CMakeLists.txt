INCLUDE_DIRECTORIES(.)

SET(LIDAR_SRC HcPointCloudData.h Hctoolkit.cpp Hctoolkit.h lidar_repair.cpp lidar_repair.h lidar_filter.cpp lidar_filter.h)
ADD_LIBRARY(lidar_toolkit ${LIDAR_SRC})
IF (WIN32)
ELSE()
    TARGET_LINK_LIBRARIES(lidar_toolkit pthread)
ENDIF()
