#include "transceiver_sx1262.h"
#include "esphome/core/log.h"

extern "C" {
#include "sx126x_hal.h"
#include "sx126x.h"
}
namespace {
static const char *TAG = "SX1262_HAL";
}

extern "C" {

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr || !self->spi_delegate_ready_())
    return SX126X_HAL_STATUS_ERROR;

  self->spi_begin_for_semtech_();
  for (uint16_t i = 0; i < command_length; i++)
    self->spi_transfer_for_semtech_(command[i]);
  for (uint16_t i = 0; i < data_length; i++)
    self->spi_transfer_for_semtech_(data[i]);
  self->spi_end_for_semtech_();
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr || !self->spi_delegate_ready_())
    return SX126X_HAL_STATUS_ERROR;

  self->spi_begin_for_semtech_();
  for (uint16_t i = 0; i < command_length; i++)
    self->spi_transfer_for_semtech_(command[i]);
  for (uint16_t i = 0; i < data_length; i++)
    data[i] = self->spi_transfer_for_semtech_(SX126X_NOP);
  self->spi_end_for_semtech_();
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr || !self->reset_pin_ready_())
    return SX126X_HAL_STATUS_ERROR;
  // Same sequence as esphome::wmbus_radio::RadioTransceiver::reset()
  self->reset_pulse_for_semtech_();
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
  auto *self = static_cast<esphome::wmbus_radio::SX1262 *>(const_cast<void *>(context));
  if (self == nullptr || !self->spi_delegate_ready_())
    return SX126X_HAL_STATUS_ERROR;

  // Semtech wakeup procedure: NSS low + send NOP (0x00)
  // In ESPHome SPIDevice, begin_transaction toggles NSS.
  self->delegate_->begin_transaction();
  self->delegate_->transfer(SX126X_NOP);
  self->spi_end_for_semtech_();
  return SX126X_HAL_STATUS_OK;
}

}  // extern "C"
