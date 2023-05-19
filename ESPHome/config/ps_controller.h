#define CUSTOM_SS_PIN 27
#define LOOP_DELAY_MS 1000

#include "esphome.h"
#include "SPI.h"


//class PSController : public Component, public CustomAPIDevice {
class PSController : public Component {
 private:
	//uint32_t motorPWMPin    = 14;
	unsigned long lastMillisSpiTransfer = 0;
	unint8_t byte_to_transfer = 0;
 public:
  void setup() override {
    // This will be called once to set up the component
    // think of it as the setup() call in Arduino
    ///pinMode(6, OUTPUT);
    //pinMode(14, OUTPUT);
    /////SPI.pins(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    //pinMode(CUSTOM_SS_PIN, OUTPUT);

    //SPI.begin();
    SPI.begin(SCK, MISO, MOSI, CUSTOM_SS_PIN);
		//begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); //from: https://github.com/espressif/arduino-esp32/blob/master/libraries/SPI/examples/SPI_Multiple_Buses/SPI_Multiple_Buses.ino
		/////SPI.setClockDivider(SPI_CLOCK_DIV8);

    // Declare a service "hello_world"
    //  - Service will be called "esphome.<NODE_NAME>_hello_world" in Home Assistant.
    //  - The service has no arguments
    //  - The function on_hello_world declared below will attached to the service.
    /////register_service(&PSController::on_hello_world, "hello_world", {"name"});

    // Declare a second service "start_washer_cycle"
    //  - Service will be called "esphome.<NODE_NAME>_start_washer_cycle" in Home Assistant.
    //  - The service has three arguments (type inferred from method definition):
    //     - cycle_duration: integer
    //     - silent: boolean
    //     - string_argument: string
    //  - The function start_washer_cycle declared below will attached to the service.
    ///register_service(&PSController::on_start_washer_cycle, "start_washer_cycle",
    ///                 {"cycle_duration", "silent", "string_argument"});

    // Subscribe to a Home Assistant state "sensor.temperature"
    //  - Each time the ESP connects or Home Assistant updates the state, the function
    //    on_state_changed will be called
    //  - The state is a string - if you want to use it as an int you must parse it manually
    ///subscribe_homeassistant_state(&PSController::on_state_changed, "sensor.temperature");
  }
	/*
  void on_hello_world(std::string name) {
    ESP_LOGD("custom", "Hello World! %s - freq = %f", name.c_str(), frequency);

    if (is_connected()) {
      // Example check to see if a client is connected
    	ESP_LOGD("custom", "Hello World -- Client is connected.");
    }
  }
	*/
  /*
 	void on_start_washer_cycle(int cycle_duration, bool silent, std::string string_argument) {
    ESP_LOGD("custom", "Starting washer cycle!");
    digitalWrite(8, HIGH);
    // do something with arguments

    // Call a homeassistant service
    call_homeassistant_service("homeassistant.service");
  }
  void on_state_changed(std::string state) {
    ESP_LOGD(TAG, "Temperature has changed to %s", state.c_str());
  }
	*/
	void loop() override {
		/*		
		SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
		SPI.transfer(0x42);
		SPI.endTransaction();
    ESP_LOGD("custom", "MOSI = %d", MOSI);
    ESP_LOGD("custom", "MISO = %d", MISO);
    ESP_LOGD("custom", "SS = %d", SS);
    ESP_LOGD("custom", "CUSTOM_SS = %d", CUSTOM_SS_PIN);
    ESP_LOGD("custom", "CLK = %d", SCK);
		*/
if(millis() > lastMillisSpiTransfer + LOOP_DELAY_MS) {
 char c;

//    ESP_LOGD("custom", "MOSI = %d", MOSI);
//    ESP_LOGD("custom", "MISO = %d", MISO);
//    ESP_LOGD("custom", "SS = %d", SS);
//    ESP_LOGD("custom", "CUSTOM_SS = %d", CUSTOM_SS_PIN);
//    ESP_LOGD("custom", "CLK = %d", SCK);

 
  ESP_LOGD("custom", "do SPI transfer...");
  ESP_LOGD("custom", "byte_to_transfer = %d", byte_to_transfer);

		//SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
		//SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
 
// enable Slave Select
  digitalWrite(CUSTOM_SS_PIN, LOW);
	// send test string
  //for (const char * p = "Hello, world!\n" ; c = *p; p++) {
    SPI.transfer(byte_to_transfer);
    //SPI.write (c);
	//}

  // disable Slave Select
  digitalWrite(CUSTOM_SS_PIN, HIGH);
  
   // ESP_LOGD("custom", "PIN_STATE = %d", pin_state ? 1 : 0);
//digitalWrite(CUSTOM_SS_PIN, pin_state ? HIGH : LOW);
	//pin_state = !pin_state;

	SPI.endTransaction();

	byte_to_transfer++;
	if(byte_to_transfer > 255) {
		byte_to_transfer = 0;
	}

  lastMillisSpiTransfer = millis();
//	delay (2000);  // 1 seconds delay 
} //millis() check 

	//	delay(1000);
	}
};
