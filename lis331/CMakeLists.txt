idf_component_register(SRCS "lis331.c"
                       INCLUDE_DIRS include
                       REQUIRES esp_driver_i2c freertos)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
