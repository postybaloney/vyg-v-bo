#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Byte pipe for the same packets USB carries. No-op unless DATA_LINK is BLE.
esp_err_t link_ble_init(void);
void link_ble_send(const void *buf, size_t n);
int link_ble_read(uint8_t *dst, size_t max, int timeout_ms);
