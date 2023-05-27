/*
 * include "desky.h"
*/
#include "ps_motor_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ps_motor_controller {

static const char *TAG = "ps_motor_controller";

/*
const char *desky_operation_to_str(DeskyOperation op) {
  switch (op) {
    case DESKY_OPERATION_IDLE:
      return "IDLE";
    case DESKY_OPERATION_RAISING:
      return "RAISING";
    case DESKY_OPERATION_LOWERING:
      return "LOWERING";
    default:
      return "UNKNOWN";
  }
}

void Desky::setup() {
  if (this->up_pin_ != nullptr)
    this->up_pin_->digital_write(false);
  if (this->down_pin_ != nullptr)
    this->down_pin_->digital_write(false);
  if (this->request_pin_ != nullptr) {
    this->request_pin_->digital_write(true);
    this->request_time_ = millis();
  }
}

void Desky::loop() {
  static int state = 0;
  static uint8_t high_byte;

  while (this->available()) {
    uint8_t c;
    int value;
    this->read_byte(&c);
    switch (state) {
     case 0:
      if (c == 1)
	state = 1;
      break;
     case 1:
      if (c == 1)
	state = 2;
      else
	state = 0;
      break;
     case 2:
      high_byte = c;
      state = 3;
      break;
     case 3:
      value = (high_byte << 8) + c;
      this->current_pos_ = value;
      if (this->height_sensor_ != nullptr)
        this->height_sensor_->publish_state(value);
      state = 0;
      break;
    }
  }

  if (this->target_pos_ >= 0) {
    if (abs(this->target_pos_ - this->current_pos_) < this->stopping_distance_)
      this->stop();
    if ((this->timeout_ >= 0) && (millis() - this->start_time_ >= this->timeout_))
      this->stop();
  }

  if ((this->request_time_ > 0) && (millis() - this->request_time_ >= 100)) {
    this->request_pin_->digital_write(false);
    this->request_time_ = 0;
  }
}

void Desky::dump_config() {
  ESP_LOGCONFIG(TAG, "Desky desk:");
  LOG_SENSOR("", "Height", this->height_sensor_);
  LOG_PIN("Up pin: ", this->up_pin_);
  LOG_PIN("Down pin: ", this->down_pin_);
  LOG_PIN("Request pin: ", this->request_pin_);
}

void Desky::move_to(int target_pos) {
  if (abs(target_pos - this->current_pos_) < this->stopping_distance_)
    return;
  if (target_pos > this->current_pos_) {
    if (this->up_pin_ == nullptr)
      return;
    this->up_pin_->digital_write(true);
    this->current_operation = DESKY_OPERATION_RAISING;
  } else {
    if (this->down_pin_ == nullptr)
      return;
    this->down_pin_->digital_write(true);
    this->current_operation = DESKY_OPERATION_LOWERING;
  }
  this->target_pos_ = target_pos;
  if (this->timeout_ >= 0)
    this->start_time_ = millis();
}

void Desky::stop() {
  this->target_pos_ = -1;
  if (this->up_pin_ != nullptr)
    this->up_pin_->digital_write(false);
  if (this->down_pin_ != nullptr)
    this->down_pin_->digital_write(false);
  this->current_operation = DESKY_OPERATION_IDLE;
}
*/

void PSMotorControllerComponent::setup() {
	this->byte_recieved = 0;
	this->spi_setup();
	//register_service(&PSMotorControllerComponent::do_calibration_routine, "do_calibration_routine", {});
}

void PSMotorControllerComponent::loop() {

}

void PSMotorControllerComponent::dump_config(){
	ESP_LOGCONFIG(TAG, "Empty SPI component");
}

//uint8_t doSeekPositionSpiCall(uint8_t psc_command) {
void PSMotorControllerComponent::do_seek_position_spi_call(uint8_t psc_command) {
  ESP_LOGD(TAG, "do_seek_position_spi_call() called");
  this->enable();
	this->write_byte(psc_command);
  delay(1);
  this->byte_recieved = this->read_byte();
  this->disable();
  ESP_LOGD(TAG, " << byte_recieved = %d", this->byte_recieved);
	//return this->byte_recieved;
}
 
void PSMotorControllerComponent::seek_position(uint8_t position) {
  ESP_LOGD(TAG, "seek_position() called");
	if (!api_is_connected()) {
  	ESP_LOGE(TAG, "API is not connected.");
		return;
	}
	switch (position) {
		case 0:
			this->do_seek_position_spi_call(2);
			break;
		case 1:
			this->do_seek_position_spi_call(3);
			break;	
		case 2:
			this->do_seek_position_spi_call(4);
			break;
		case 3:
			this->do_seek_position_spi_call(5);
			break;
		case 4:
			this->do_seek_position_spi_call(6);
			break;
		case 5:
			this->do_seek_position_spi_call(7);
			break;
		case 6:
			this->do_seek_position_spi_call(8);
			break;
		case 7:
			this->do_seek_position_spi_call(9);
			break;
		case 8:
			this->do_seek_position_spi_call(10);
			break;
		case 9:
			this->do_seek_position_spi_call(11);
			break;
		case 10:
			this->do_seek_position_spi_call(12);
			break;
		default:
  		ESP_LOGE(TAG, "Unknown seek position requested.");
	}
}

void PSMotorControllerComponent::do_calibration_routine() {
  if (api_is_connected()) {
  	//ESP_LOGCONFIG(TAG, "Called do_calibration_routine()");
  	ESP_LOGD(TAG, "Called do_calibration_routine()");
  	//ESP_LOGD("custom", "Do calibration routine called");
		/////SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
		//enable Slave Select
  	/////digitalWrite(CUSTOM_SS_PIN, LOW);
  	this->enable();
    /////SPI.transfer(1);
		this->write_byte(1);
   	delay(1);
   	/////byte_recieved = SPI.transfer(0);
  	this->byte_recieved = this->read_byte();
		//disable Slave Select
  	/////digitalWrite(CUSTOM_SS_PIN, HIGH);
  	this->disable();
		/////SPI.endTransaction();
  	ESP_LOGD(TAG, " << byte_recieved = %d", this->byte_recieved);
   }
}

}  // namespace desky
}  // namespace esphome
