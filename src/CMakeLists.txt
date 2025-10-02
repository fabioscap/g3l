add_library(utilities STATIC g3l.cpp)
target_include_directories(utilities PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utilities PUBLIC cxxopts KortexApiCpp tinyxml2::tinyxml2)



add_executable(main main.cpp)
target_link_libraries(main
utilities
)