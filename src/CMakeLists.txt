INCLUDE_DIRECTORIES(${CAMSENSE_SOURCE_DIR}/src/base)
LINK_DIRECTORIES(${CAMSENSE_SOURCE_DIR}/src/base)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

ADD_SUBDIRECTORY(base)

ADD_EXECUTABLE (sdk_demo sdk_demo.cpp)
TARGET_LINK_LIBRARIES (sdk_demo lidar)


ADD_EXECUTABLE (sdk_V3_demo sdk_V3_demo.cpp LidarTest.cpp)
TARGET_LINK_LIBRARIES (sdk_V3_demo lidar)

ADD_EXECUTABLE (sdk_scanData sdk_scanData.cpp)
TARGET_LINK_LIBRARIES (sdk_scanData lidar)

ADD_EXECUTABLE (BS_Demo BS_Demo.cpp LidarTest.cpp)

if(UNIX)

if(CMAKE_TOOLCHAIN_FILE)
TARGET_LINK_LIBRARIES (BS_Demo lidar libChargingPoint_arm.a)
else()
TARGET_LINK_LIBRARIES (BS_Demo lidar libChargingPoint_x64.a)
endif()

endif(UNIX)


if (MSVC)
    target_link_libraries(BS_Demo lidar ChargingPoint)
endif (MSVC)