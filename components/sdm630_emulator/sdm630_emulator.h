#pragma once

#include <array>
#include <memory>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/socket/socket.h"
#include "esphome/core/component.h"
#include "sdm630_protocol.h"

namespace esphome {
namespace sdm630_emulator {

class SDM630Emulator : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_port(uint16_t port) { this->port_ = port; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }
  void set_max_age(uint32_t ms) { this->max_age_ms_ = ms; }
  void set_nominal_voltage(float v) { this->nominal_voltage_ = v; }
  void set_voltage(uint8_t phase, sensor::Sensor *s) { this->voltage_[phase] = s; }
  void set_power_delivered(uint8_t phase, sensor::Sensor *s) { this->power_delivered_[phase] = s; }
  void set_power_returned(uint8_t phase, sensor::Sensor *s) { this->power_returned_[phase] = s; }

 protected:
  static constexpr uint8_t MAX_CLIENTS = 3;
  static constexpr uint32_t CLIENT_IDLE_TIMEOUT_MS = 60000;

  struct Client {
    std::unique_ptr<socket::Socket> sock;
    uint8_t buf[MAX_FRAME_LEN];
    size_t len{0};
    uint32_t last_active{0};
  };

  void fail_(const char *what);
  void accept_clients_();
  void service_client_(Client &c, uint32_t now);
  void close_client_(Client &c);
  bool collect_values_(Values *out);

  std::unique_ptr<socket::ListenSocket> listen_;
  std::array<Client, MAX_CLIENTS> clients_;

  sensor::Sensor *voltage_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *power_delivered_[3]{nullptr, nullptr, nullptr};
  sensor::Sensor *power_returned_[3]{nullptr, nullptr, nullptr};

  uint16_t port_{502};
  uint8_t unit_id_{1};
  uint32_t max_age_ms_{10000};
  float nominal_voltage_{230.0f};

  uint32_t last_update_ms_{0};
  bool has_data_{false};
  bool stale_{true};
};

}  // namespace sdm630_emulator
}  // namespace esphome
