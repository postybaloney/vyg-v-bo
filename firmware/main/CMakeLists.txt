set(_reqs esp_driver_spi esp_driver_gpio esp_driver_i2c esp_driver_usb_serial_jtag esp_timer esp_hw_support esp_adc esp_driver_i2s)
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/config.h" _data_link REGEX "^#define DATA_LINK ")
if(_data_link MATCHES "DATA_LINK_BLE")
    list(APPEND _reqs bt nvs_flash)
endif()

idf_component_register(SRCS "main.c" "ads1299.c" "fpga.c" "impedance.c" "audio.c" "link_ble.c"
                       INCLUDE_DIRS "."
                       REQUIRES ${_reqs})