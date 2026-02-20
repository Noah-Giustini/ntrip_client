cmake_minimum_required(VERSION 3.10)
project(ntrip_client)

set(CMAKE_CXX_STANDARD 17)

add_executable(ntrip_client
    main.cpp
    ntrip_client.cpp
    nmea.cpp
    serial.cpp
)

# nlohmann_json single-header
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ntrip_client PRIVATE nlohmann_json)

# pthread for threading
if(UNIX)
    target_link_libraries(ntrip_client PRIVATE pthread)
endif()
