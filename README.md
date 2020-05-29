# esp32-lora-sensor
IoT-enable your ancient Rubbermaid mailbox with an ESP32, LoRa radio, and contact closure sensor.

My mailbox isn't that close, and I wanted a way to see when it was opened. This project uses an ESP32 w/ contact sensor to detect when the mailbox door becomes open/closed, a LoRa radio to send a quick JSON burst, and then deep sleeps until the door opens again. 

You'll need a receiver to pick up the message and do something with it on the far end. I'll link mine here when done.

It heavily borrows door state logic from [acbrandao's esp32_mailbox](http://www.abrandao.com/2018/04/arduino_esp32_battery_wifi_door_mailbox_sensor/) project, with some improvements for detecting a stuck door condition.

## Getting Started

#### Hardware

It only requires a couple things:
* [ESP32 development board w/ LoRa radio](https://usa.banggood.com/LILYGO-TTGO-2Pcs-ESP32-SX1276-LoRa-915MHz-bluetooth-WI-FI-Internet-Antenna-Development-Board-p-1466793.html)
* [Contact closure sensor](https://www.adafruit.com/product/375)
* 18650 battery (or any battery that can power the ESP32)
* Wires, solder, soldering iron, and some kind of weatherproof box
* Micro USB cable

#### Software

* VSCode
* PlatformIO

PlatformIO + VSCode will handle all your library dependencies so you should be able to just build.

You'll likely need to install the [CP210x drivers](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) so your computer recognizes the chipset on the ESP32.

### Configuring things

Optionally you can tweak these settings to better suit your needs:

```
#define IS_DEBUG false // enable Serial.println statements

...

#define SYNCWORD 0xF3  // used to make sure our receiver only listens to our packets

...
// Mailbox config
#define PROJECT_NAME "rad-esp32-lora-sender" // What do we call this thing?
#define LED_ENABLED true                       // Turn on the on-board LED when transmitting status
#define MAX_OPENDOOR_TIME 30000                // default 30s in milliseconds how long to wait while door is open to consider it stuck open
#define MAX_STUCK_BOOT_COUNT 5                 // If the door is stuck for more than x times let's switch to timer interrupt to save battery
#define TIMER_SLEEP_MICROSECS 1800 * 1000000   // when on timer interrupt how long to sleep in seconds * microseconds

// Define the contact closure input pin and initial state (Depends on your sensor - N/C or N/O )
gpio_num_t doorSensorPIN = GPIO_NUM_34;        // GPIO for the contact closure
const int GPIO_DOOR_CLOSED_STATE = HIGH;       // Default state when the reed and magnet are next to each other
```

Save, build, and deploy. 🎉

## Installing and Testing 

  *  Attach your ESP32 board with your USB cable
  *  Build the project using the PlatformIO extension
  *  Upload and Monitor to push the new firmware and watch the output
  *  Move the magnet closer/further from the sensor :)

## Contributing

I like PRs :)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
