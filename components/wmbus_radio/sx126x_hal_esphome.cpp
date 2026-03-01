#include "transceiver_sx1262.h"
#include "sx126x_hal.h"
#include "esphome/core/log.h"

namespace {
static const char *TAG = "SX1262_HAL";
}

extern "C" {

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr)
    return SX126X_HAL_STATUS_ERROR;

  return self->semtech_hal_write(command, command_length, data, data_length) ? SX126X_HAL_STATUS_OK
                                                                             : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr)
    return SX126X_HAL_STATUS_ERROR;

  return self->semtech_hal_read(command, command_length, data, data_length) ? SX126X_HAL_STATUS_OK
                                                                            : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr)
    return SX126X_HAL_STATUS_ERROR;

  return self->semtech_hal_reset() ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr)
    return SX126X_HAL_STATUS_ERROR;

  return self->semtech_hal_wakeup() ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

}  // extern "C"