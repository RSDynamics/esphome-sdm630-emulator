#include "sdm630_emulator.h"

#include <cerrno>
#include <cinttypes>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sdm630_emulator {

static const char *const TAG = "sdm630_emulator";

void SDM630Emulator::fail_(const char *what) {
  ESP_LOGE(TAG, "Socket %s failed: errno %d", what, errno);
  this->listen_.reset();
  this->mark_failed();
}

void SDM630Emulator::setup() {
  // Freshness is tracked on the L1 import sensor: the DSMR component republishes it with every
  // telegram (about once per second), even if the value did not change.
  this->power_delivered_[0]->add_on_state_callback([this](float) {
    this->last_update_ms_ = millis();
    this->has_data_ = true;
  });

  this->listen_ = socket::socket_ip_loop_monitored(SOCK_STREAM, 0);
  if (this->listen_ == nullptr) {
    this->fail_("creation");
    return;
  }
  int enable = 1;
  this->listen_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  if (this->listen_->setblocking(false) != 0) {
    this->fail_("nonblocking");
    return;
  }
  struct sockaddr_storage server;
  socklen_t sl = socket::set_sockaddr_any((struct sockaddr *) &server, sizeof(server), this->port_);
  if (sl == 0) {
    this->fail_("set sockaddr");
    return;
  }
  if (this->listen_->bind((struct sockaddr *) &server, sl) != 0) {
    this->fail_("bind");
    return;
  }
  if (this->listen_->listen(MAX_CLIENTS) != 0) {
    this->fail_("listen");
    return;
  }
}

void SDM630Emulator::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SDM630 Modbus TCP emulator:\n"
                "  Port: %u\n"
                "  Unit ID: %u\n"
                "  Max data age: %" PRIu32 " ms\n"
                "  Nominal voltage: %.0f V",
                this->port_, this->unit_id_, this->max_age_ms_, this->nominal_voltage_);
}

bool SDM630Emulator::collect_values_(Values *out) {
  bool fresh = this->has_data_ && (millis() - this->last_update_ms_) <= this->max_age_ms_;
  if (fresh != !this->stale_) {
    this->stale_ = !fresh;
    if (this->stale_) {
      ESP_LOGW(TAG, "No fresh meter data for %" PRIu32 " ms: answering Modbus reads with an error", this->max_age_ms_);
    } else {
      ESP_LOGI(TAG, "Meter data is fresh again");
    }
  }
  if (!fresh) return false;

  float volt[3], delivered[3], returned[3];
  for (int i = 0; i < 3; i++) {
    volt[i] = this->voltage_[i] != nullptr ? this->voltage_[i]->state : NAN;
    delivered[i] = this->power_delivered_[i] != nullptr ? this->power_delivered_[i]->state : NAN;
    returned[i] = this->power_returned_[i] != nullptr ? this->power_returned_[i]->state : NAN;
  }
  // L1 import is the freshness sensor: if it has no value, do not pretend to have data.
  // Phases whose import sensor has no value (L2/L3 on a single-phase connection) read all zero.
  if (std::isnan(delivered[0])) return false;
  *out = compute_values(volt, delivered, returned, this->nominal_voltage_);
  return true;
}

void SDM630Emulator::close_client_(Client &c) {
  if (c.sock != nullptr) {
    c.sock->close();
    c.sock.reset();
  }
  c.len = 0;
}

void SDM630Emulator::accept_clients_() {
  while (true) {
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    auto sock = this->listen_->accept_loop_monitored((struct sockaddr *) &source_addr, &addr_len);
    if (!sock) break;
    sock->setblocking(false);

    char peername[socket::SOCKADDR_STR_LEN];
    sock->getpeername_to(peername);

    // Pick a free slot, otherwise evict the client that has been quiet the longest
    // (a charger that reconnects may leave a half-open connection behind).
    Client *slot = nullptr;
    for (auto &c : this->clients_) {
      if (c.sock == nullptr) {
        slot = &c;
        break;
      }
    }
    if (slot == nullptr) {
      slot = &this->clients_[0];
      for (auto &c : this->clients_) {
        if (c.last_active < slot->last_active) slot = &c;
      }
      this->close_client_(*slot);
    }
    ESP_LOGD(TAG, "Accepted %s", peername);
    slot->sock = std::move(sock);
    slot->len = 0;
    slot->last_active = millis();
  }
}

void SDM630Emulator::service_client_(Client &c, uint32_t now) {
  if (now - c.last_active > CLIENT_IDLE_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Closing idle client");
    this->close_client_(c);
    return;
  }

  ssize_t n = c.sock->read(c.buf + c.len, sizeof(c.buf) - c.len);
  if (n == 0) {
    this->close_client_(c);
    return;
  }
  if (n < 0) {
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
      this->close_client_(c);
      return;
    }
  } else {
    c.len += static_cast<size_t>(n);
    c.last_active = now;
  }

  while (c.len >= 6) {
    int total = frame_length(c.buf, c.len);
    if (total < 0 || static_cast<size_t>(total) > sizeof(c.buf)) {
      ESP_LOGW(TAG, "Invalid Modbus frame, closing connection");
      this->close_client_(c);
      return;
    }
    if (c.len < static_cast<size_t>(total)) break;  // wait for the rest of the frame

    if (total >= 12) {
      ESP_LOGV(TAG, "Request unit=%u fc=%u addr=%u count=%u", c.buf[6], c.buf[7],
               static_cast<unsigned>((c.buf[8] << 8) | c.buf[9]), static_cast<unsigned>((c.buf[10] << 8) | c.buf[11]));
    }

    Values values;
    bool ok = this->collect_values_(&values);
    uint8_t resp[MAX_RESPONSE_LEN];
    size_t rl = build_response(c.buf, static_cast<size_t>(total), ok ? &values : nullptr, this->unit_id_, resp);
    if (rl > 0) {
      ssize_t w = c.sock->write(resp, rl);
      if (w != static_cast<ssize_t>(rl)) {
        ESP_LOGW(TAG, "Short write to client, closing connection");
        this->close_client_(c);
        return;
      }
    }
    std::memmove(c.buf, c.buf + total, c.len - static_cast<size_t>(total));
    c.len -= static_cast<size_t>(total);
  }
}

void SDM630Emulator::loop() {
  if (this->listen_ == nullptr) return;
  if (this->listen_->ready()) this->accept_clients_();
  uint32_t now = millis();
  for (auto &c : this->clients_) {
    if (c.sock != nullptr) this->service_client_(c, now);
  }
}

}  // namespace sdm630_emulator
}  // namespace esphome
