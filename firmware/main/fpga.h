#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// I2C master on the FPGA/haptic bus. Does not probe the slaves.
esp_err_t fpga_init(void);

// Chip-select / stim register on the FPGA. mask is FPGA_CS_NONE / EMG / EEG,
// or an impedance ramp command (see config.h). Absolute write.
esp_err_t fpga_cs(uint8_t mask);

esp_err_t fpga_set_start(bool run);
