add_library(utilities STATIC utilities.cpp)
target_include_directories(utilities PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utilities PUBLIC cxxopts)


add_executable(main main.cpp)
target_link_libraries(main
KortexApiCpp
utilities
)