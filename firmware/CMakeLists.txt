
cmake_minimum_required(VERSION 3.16)

set(_audio_cfg "${CMAKE_CURRENT_LIST_DIR}/main/config.h")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_audio_cfg}")
file(STRINGS "${_audio_cfg}" _data_link REGEX "^#define DATA_LINK ")
if(_data_link MATCHES "DATA_LINK_BLE")
    file(READ "${CMAKE_CURRENT_LIST_DIR}/sdkconfig" _sdk_old)
    string(REPLACE "# CONFIG_BT_ENABLED is not set" "CONFIG_BT_ENABLED=y" _sdk "${_sdk_old}")
    if(NOT _sdk STREQUAL _sdk_old)
        file(WRITE "${CMAKE_CURRENT_LIST_DIR}/sdkconfig" "${_sdk}")
    endif()
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.ble")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(eeg_fw)
