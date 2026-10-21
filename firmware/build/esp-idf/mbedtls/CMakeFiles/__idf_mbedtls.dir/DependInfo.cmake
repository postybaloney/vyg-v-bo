
# Consider dependencies only in project.
set(CMAKE_DEPENDS_IN_PROJECT_ONLY OFF)

# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "/Users/ivanbokarev/code/vyg-v0/firmware/build/x509_crt_bundle.S" "/Users/ivanbokarev/code/vyg-v0/firmware/build/esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "ESP_PLATFORM"
  "ESP_PSA_ITS_AVAILABLE"
  "IDF_VER=\"v6.1-dev-4631-g9bbb6446b8\""
  "MBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\""
  "MBEDTLS_MAJOR_VERSION=4"
  "SOC_MMU_PAGE_SIZE=CONFIG_MMU_PAGE_SIZE"
  "SOC_XTAL_FREQ_MHZ=CONFIG_XTAL_FREQ"
  "TF_PSA_CRYPTO_USER_CONFIG_FILE=\"mbedtls/esp_config.h\""
  "_GLIBCXX_HAVE_POSIX_SEMAPHORE"
  "_GLIBCXX_USE_POSIX_SEMAPHORE"
  "_GNU_SOURCE"
  "_POSIX_READER_WRITER_LOCKS"
  "__STDC_WANT_LIB_EXT1__=0"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "config"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/port/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/library"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/core"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/extras"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/esp_crt_bundle/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/port/psa_driver/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_libc/platform_include"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/config/include"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/config/include/freertos"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/config/riscv/include"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/FreeRTOS-Kernel/include"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/FreeRTOS-Kernel/portable/riscv/include"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/FreeRTOS-Kernel/portable/riscv/include/freertos"
  "/Users/ivanbokarev/esp/esp-idf/components/freertos/esp_additions/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/include/soc"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/ldo/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/debug_probe/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/etm/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/mspi/mspi_timing_tuning/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/mspi/mspi_timing_tuning/tuning_scheme_impl/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/mspi/mspi_intr/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/power_supply/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/modem/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/include/soc/esp32c3"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/port/esp32c3/."
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hw_support/port/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/heap/include"
  "/Users/ivanbokarev/esp/esp-idf/components/heap/tlsf"
  "/Users/ivanbokarev/esp/esp-idf/components/log/include"
  "/Users/ivanbokarev/esp/esp-idf/components/soc/include"
  "/Users/ivanbokarev/esp/esp-idf/components/soc/esp32c3"
  "/Users/ivanbokarev/esp/esp-idf/components/soc/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/soc/esp32c3/register"
  "/Users/ivanbokarev/esp/esp-idf/components/hal/platform_port/include"
  "/Users/ivanbokarev/esp/esp-idf/components/hal/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/hal/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_rom/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_rom/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_rom/esp32c3/include/esp32c3"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_rom/esp32c3"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_common/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_system/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_system/port/soc"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_system/port/include/riscv"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_system/port/include/private"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_stdio/include"
  "/Users/ivanbokarev/esp/esp-idf/components/riscv/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_gpio/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_gpio/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_usb/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_usb/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_pmu/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_pmu/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_regi2c/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_regi2c/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_ana_conv/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_ana_conv/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_dma/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_dma/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/include"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/include/apps"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/lwip/src/include"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/port/include"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/port/freertos/include"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/port/esp32xx/include"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/port/esp32xx/include/arch"
  "/Users/ivanbokarev/esp/esp-idf/components/lwip/port/esp32xx/include/sys"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_security/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_security/esp32c3/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_hal_security/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_pm/include"
  "/Users/ivanbokarev/esp/esp-idf/components/esp_driver_dma/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include"
  "esp-idf/mbedtls/mbedtls/tf-psa-crypto/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/drivers/everest/include"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/drivers/p256-m/p256-m"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/mbedtls/tf-psa-crypto/drivers/pqcp/include"
  )

# The set of dependency files which are needed:
set(CMAKE_DEPENDS_DEPENDENCY_FILES
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/esp_crt_bundle/esp_crt_bundle.c" "esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj" "gcc" "esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj.d"
  "/Users/ivanbokarev/esp/esp-idf/components/mbedtls/port/esp_mem.c" "esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/port/esp_mem.c.obj" "gcc" "esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/port/esp_mem.c.obj.d"
  )

# Targets to which this target links which contain Fortran sources.
set(CMAKE_Fortran_TARGET_LINKED_INFO_FILES
  )

# Targets to which this target links which contain Fortran sources.
set(CMAKE_Fortran_TARGET_FORWARD_LINKED_INFO_FILES
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
