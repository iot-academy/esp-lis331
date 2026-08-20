set(lis331_requires "esp_driver_i2c freertos")

idf_component_register(SRCS "lis331.c"
                       INCLUDE_DIRS include
                       REQUIRES ${lis331_requires})

# ESP-IDF resolves REQUIRES during the early expansion pass, before the
# component's own Kconfig symbols are generated. A plain REQUIRES entry gated on
# CONFIG_LIS331_USE_I2C_BUS would therefore not be picked up. idf_component_optional_requires()
# adds the link/include dependency at add_subdirectory() time, when the value is
# already known (same pattern as components/esp_netif in IDF v6.0.2).
if(CONFIG_LIS331_USE_I2C_BUS)
    idf_component_optional_requires(PUBLIC espressif__i2c_bus)
endif()

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})