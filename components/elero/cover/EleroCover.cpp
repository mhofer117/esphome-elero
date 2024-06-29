#include "EleroCover.h"
#include "esphome/core/log.h"

namespace esphome {
namespace elero {

using namespace esphome::cover;

static const char *const TAG = "elero.cover";

const float TILT_OPEN = 1.0f;
const float TILT_CLOSED = 0.0f;

void EleroCover::dump_config() {
  LOG_COVER("", "Elero Cover", this);
}

void EleroCover::setup() {
  this->parent_->register_cover(this);
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    if ((this->open_duration_ > 0) && (this->close_duration_ > 0)) {
      this->position = 0.5f;
    }
    if ((this->tilt_open_duration_ > 0) && (this->tilt_close_duration_ > 0)) {
      this->tilt = 0.5f;
    }
  }
}

void EleroCover::loop() {
  uint32_t intvl = this->poll_intvl_;
  uint32_t now = millis();
  if(this->current_operation != COVER_OPERATION_IDLE) {
    if((now - ELERO_TIMEOUT_MOVEMENT) < this->movement_start_) // do not poll frequently for an extended period of time
      intvl = ELERO_POLL_INTERVAL_MOVING;
  }

  if((now > this->poll_offset_) && (now - this->poll_offset_ - this->last_poll_) > intvl) {
    ESP_LOGV(TAG, "'%s': Polling blind", this->name_.c_str());
    this->commands_to_send_.push(this->command_check_);
    this->last_poll_ = now - this->poll_offset_;
  }

  this->handle_commands(now);

  if((this->current_operation != COVER_OPERATION_IDLE) && (((this->open_duration_ > 0) && (this->close_duration_ > 0)) || (this->tilt_open_duration_ > 0) && (this->tilt_close_duration_ > 0))) {
    this->recompute_position();
    if(this->is_at_target()) {
      // We don't want to send a stop command for completely open or close,
      // this is handled by the cover
      if( !(this->target_position_ == COVER_OPEN && this->target_tilt_ == TILT_OPEN) &&
          !(this->target_position_ == COVER_CLOSED && this->target_tilt_ == TILT_CLOSED)) {
        ESP_LOGV(TAG, "'%s': Target position reached, sending STOP command", this->name_.c_str());
        this->commands_to_send_.push(this->command_stop_);
      }
      this->current_operation = COVER_OPERATION_IDLE;
      this->publish_state();
      this->last_publish_ = now;
    }

    // Publish position every second
    if(now - this->last_publish_ > 1000) {
      this->publish_state(false);
      this->last_publish_ = now;
    }
  }
}

bool EleroCover::is_at_target() {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return (this->position >= this->target_position_) && (this->tilt >= this->target_tilt_);
    case COVER_OPERATION_CLOSING:
      return (this->position <= this->target_position_) && (this->tilt <= this->target_tilt_);
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void EleroCover::handle_commands(uint32_t now) {
  if((now - this->last_command_) > ELERO_DELAY_SEND_PACKETS) {
    if(this->commands_to_send_.size() > 0) {
      this->command_.payload[4] = this->commands_to_send_.front();
      if(this->parent_->send_command(&this->command_)) {
        this->send_packets_++;
        this->send_retries_ = 0;
        if(this->send_packets_ >= ELERO_SEND_PACKETS) {
          this->commands_to_send_.pop();
          this->send_packets_ = 0;
          this->increase_counter();
        }
      } else {
        ESP_LOGD(TAG, "Retry #%d for blind 0x%02x", this->send_retries_, this->command_.blind_addr);
        this->send_retries_++;
        if(this->send_retries_ > ELERO_SEND_RETRIES) {
          ESP_LOGE(TAG, "Hit maximum number of retries, giving up.");
          this->send_retries_ = 0;
          this->commands_to_send_.pop();
        }
      }
      this->last_command_ = now;
    }
  }
}

float EleroCover::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits EleroCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  if((this->open_duration_ > 0) && (this->close_duration_ > 0))
    traits.set_supports_position(true);
  else
    traits.set_supports_position(false);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(true);
  traits.set_supports_tilt(this->supports_tilt_);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGV(TAG, "Got state: 0x%02x for blind 0x%02x", state, this->command_.blind_addr);
  float pos = this->position;
  float current_tilt = this->tilt;
  CoverOperation op = this->current_operation;

  switch(state) {
  case ELERO_STATE_TOP:
    pos = COVER_OPEN;
    current_tilt = TILT_OPEN;
    break;
  case ELERO_STATE_BOTTOM:
    pos = COVER_CLOSED;
    current_tilt = TILT_CLOSED;
    break;
  case ELERO_STATE_START_MOVING_UP:
  case ELERO_STATE_MOVING_UP:
    op = COVER_OPERATION_OPENING;
    break;
  case ELERO_STATE_START_MOVING_DOWN:
  case ELERO_STATE_MOVING_DOWN:
    op = COVER_OPERATION_CLOSING;
    break;
  case ELERO_STATE_BLOCKING:
  case ELERO_STATE_OVERHEATED:
  case ELERO_STATE_TIMEOUT:
    op = COVER_OPERATION_IDLE;
    break;
  case ELERO_STATE_INTERMEDIATE:
  case ELERO_STATE_TILT:
  case ELERO_STATE_STOPPED:
  case ELERO_STATE_TOP_TILT:
  case ELERO_STATE_BOTTOM_TILT:
  case ELERO_STATE_OFF:
  case ELERO_STATE_ON:
  case ELERO_STATE_UNKNOWN:
  default:
    // do nothing, keep current state
    break;
  }

  if((pos != this->position) || (op != this->current_operation) || (current_tilt != this->tilt)) {
    this->position = pos;
    this->tilt = current_tilt;
    this->current_operation = op;
    this->publish_state();
  }
}

void EleroCover::increase_counter() {
  if(this->command_.counter == 0xff)
    this->command_.counter = 1;
  else
    this->command_.counter += 1;
}

void EleroCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->start_movement(COVER_OPERATION_IDLE);
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    this->target_position_ = pos;
    if((pos > this->position) || (pos == COVER_OPEN)) {
      this->target_tilt_ = TILT_OPEN;
      this->start_movement(COVER_OPERATION_OPENING);
    } else {
      this->target_tilt_ = TILT_CLOSED;
      this->start_movement(COVER_OPERATION_CLOSING);
    }
  }
  if (call.get_tilt().has_value()) {
    auto tilt = *call.get_tilt();
    this->target_tilt_ = tilt;
    this->target_position_ = this->position;
    if(tilt > this->tilt) {
      this->start_movement(COVER_OPERATION_OPENING);
    } else if (tilt < this->tilt) {
      this->start_movement(COVER_OPERATION_CLOSING);
    } else {
      ESP_LOGD(TAG, "'%s': Tilt already set", this->name_.c_str());
    }
  }
  if (call.get_toggle().has_value()) {
    if(this->current_operation != COVER_OPERATION_IDLE) {
      this->start_movement(COVER_OPERATION_IDLE);
    } else {
      if(this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
        this->target_position_ = COVER_OPEN;
        this->target_tilt_ = TILT_OPEN;
        this->start_movement(COVER_OPERATION_OPENING);
      } else {
        this->target_position_ = COVER_CLOSED;
        this->target_tilt_ = TILT_CLOSED;
        this->start_movement(COVER_OPERATION_CLOSING);
      }
    }
  }
}

// FIXME: Most of this should probably be moved to the
// handle_commands function to only publish a new state
// if at least the transmission was successful
void EleroCover::start_movement(CoverOperation dir) {
  switch(dir) {
    case COVER_OPERATION_OPENING:
      ESP_LOGV(TAG, "'%s': Sending OPEN command", this->name_.c_str());
      this->commands_to_send_.push(this->command_up_);
      this->last_operation_ = COVER_OPERATION_OPENING;
    break;
    case COVER_OPERATION_CLOSING:
      ESP_LOGV(TAG, "'%s': Sending CLOSE command", this->name_.c_str());
      this->commands_to_send_.push(this->command_down_);
      this->last_operation_ = COVER_OPERATION_CLOSING;
    break;
    case COVER_OPERATION_IDLE:
      ESP_LOGV(TAG, "'%s': Sending STOP command", this->name_.c_str());
      this->commands_to_send_.push(this->command_stop_);
    break;
  }

  if(dir == this->current_operation)
    return;

  this->current_operation = dir;
  this->movement_start_ = millis();
  this->last_poll_ = millis();
  this->last_recompute_time_ = millis();
  this->publish_state();
  this->last_publish_ = millis();
}

void EleroCover::recompute_position() {
  if(this->current_operation == COVER_OPERATION_IDLE)
    return;


  float dir;
  float action_dur;
  float tilt_dur;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      tilt_dur = this->tilt_open_duration_;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      tilt_dur = this->tilt_close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();
  if (action_dur > 0 ) {
    this->position += dir * (now - this->last_recompute_time_) / action_dur;
    this->position = clamp(this->position, 0.0f, 1.0f);
  }
  if (tilt_dur > 0 ) {
    this->tilt += dir * (now - this->last_recompute_time_) / tilt_dur;
    this->tilt = clamp(this->tilt, 0.0f, 1.0f);
  }

  this->last_recompute_time_ = now;

}

} // namespace elero
} // namespace esphome
