include(FetchContent)

FetchContent_Declare(
  kinova_binary_api
  URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.3.0/linux_x86-64_x86_gcc.zip
  URL_HASH MD5=9a3bfd4f408d629276c35b8fde06e27c
)

FetchContent_MakeAvailable(kinova_binary_api)

# Create IMPORTED target
add_library(KortexApiCpp STATIC IMPORTED GLOBAL)

# Full path to .a file
set(_kortex_lib "${kinova_binary_api_SOURCE_DIR}/lib/release/libKortexApiCpp.a")

# Apply to all configs
set_target_properties(KortexApiCpp PROPERTIES
  IMPORTED_LOCATION "${_kortex_lib}"
  IMPORTED_LOCATION_RELEASE "${_kortex_lib}"
  IMPORTED_LOCATION_DEBUG "${_kortex_lib}"
)

# Public headers
target_include_directories(KortexApiCpp INTERFACE
  "${kinova_binary_api_SOURCE_DIR}/include"
)

# Definitions
target_compile_definitions(KortexApiCpp INTERFACE
  _OS_UNIX=1
)

# Dependencies
target_link_libraries(KortexApiCpp INTERFACE pthread)


target_include_directories(KortexApiCpp INTERFACE
  ${kinova_binary_api_SOURCE_DIR}/include/client
  ${kinova_binary_api_SOURCE_DIR}/include/common
  ${kinova_binary_api_SOURCE_DIR}/include/google
  ${kinova_binary_api_SOURCE_DIR}/include/client_stubs
  ${kinova_binary_api_SOURCE_DIR}/include/messages
)


