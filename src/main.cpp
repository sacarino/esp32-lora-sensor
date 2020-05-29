// ** Sender code **

// show debug output
#define IS_DEBUG false // enable Serial.println statements

#include <LoRa.h>
#include <Wire.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"
#include <ArduinoJson.h>

// LORA config
#define SCK 5   // GPIO5  -- SX1278's SCK
#define MISO 19 // GPIO19 -- SX1278's MISnO
#define MOSI 27 // GPIO27 -- SX1278's MOSI
#define SS 18   // GPIO18 -- SX1278's CS
#define RST 14  // GPIO14 -- SX1278's RESET
#define DI0 26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 915E6 // xmit freq. 915E6 is 915Mhz, 433E6 is 433Mhz
#define ERROR "ERROR: Could not find a valid LoRa transmitter. Check pin defs and wiring."

/* LoRa transceiver modules listen to packets within its range. It doesn’t matter where 
 * the packets come from. To ensure you only receive packets from your sender, you can 
 * set a sync word (ranges from 0 to 0xFF).
 */
#define SYNCWORD 0xF3

// Mailbox config
#define PROJECT_NAME "homie-esp32-lora-sender" // What do we call this thing?
#define LED_ENABLED true                       // Turn on the on-board LED when transmitting status
#define MAX_OPENDOOR_TIME 30000                // default 30s in milliseconds how long to wait while door is open to consider it stuck open
#define MAX_STUCK_BOOT_COUNT 5                 // If the door is stuck for more than x times let's switch to timer interrupt to save battery
#define TIMER_SLEEP_MICROSECS 1800 * 1000000   // when on timer interrupt how long to sleep in seconds * microseconds

// sensor/door_status messages for open closed and stuck
#define message_door_open "MAILBOX_OPEN"
#define message_door_closed "MAILBOX_CLOSED"
#define message_door_stuck "MAILBOX_STUCK_OPEN"

// Define the contact closure PIN and initial state (Depends on your reed sensor - N/C or N/O )
gpio_num_t doorSensorPIN = GPIO_NUM_34;                   // GPIO for the contact closure
gpio_num_t GPIO_INPUT_IO_TRIGGER = doorSensorPIN;         // Alias??
const int GPIO_DOOR_CLOSED_STATE = HIGH;                  // Default state when the reed and magnet are next to each other
const int GPIO_DOOR_OPEN_STATE = !GPIO_DOOR_CLOSED_STATE; // "Open" is oposite of closed
const int GPIO_DOOR_STUCK_STATE = -1;                     // Stuck open state when MAC_OPENDOOR_TIME exceeded

// RTC Memory attribute is retained across resets
RTC_DATA_ATTR long bootCount = 0;
RTC_DATA_ATTR long stuckbootCount = 0;    // if the door is stuck open increment this counter
RTC_DATA_ATTR long last_doorState = 0;    // will store last door state to detect stuck open
RTC_DATA_ATTR long time_awake_millis = 0; // total awake time in milliseconds - useful for tracking how long battery lasts
RTC_DATA_ATTR bool isStuck = false;

long currentMillis;        // timer start of awake
int doorState = HIGH;      // maintains the door sensor state usually LOW= 0 or 1
int start_time = 0;        // timing for when esp32 is active (not sleeping)
char total_time_awake[21]; // timing holds string h:m:s for time door is opened.

// Function that prints what woke the ESP32
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case 1:
        if (IS_DEBUG)
            Serial.println("\nWakeup caused by external signal using RTC_IO");
        break;
    case 2:
        if (IS_DEBUG)
            Serial.println("\nWakeup caused by external signal using RTC_CNTL");
        break;
    case 3:
        if (IS_DEBUG)
            Serial.println("\nWakeup caused by timer");
        break;
    case 4:
        if (IS_DEBUG)
            Serial.println("\nWakeup caused by touchpad");
        break;
    case 5:
        if (IS_DEBUG)
            Serial.println("\nWakeup caused by ULP program");
        break;
    default:
        if (IS_DEBUG)
            Serial.println("\nWakeup was not caused by deep sleep");
        break;
    }
}

String humanizedStatus(int doorState) {
    switch (doorState) 
    {
    case GPIO_DOOR_OPEN_STATE:
        return message_door_open;
    case GPIO_DOOR_CLOSED_STATE:
        return message_door_closed;
    case GPIO_DOOR_STUCK_STATE:
        return message_door_stuck;
    default:
        return "UNKNOWN_ERROR";
    }
}

void xmit(int doorState, String message)
{
    const int capacity = JSON_OBJECT_SIZE(6);
    StaticJsonDocument<capacity> doc;

    doc["sensor"]["door"] = doorState;
    doc["sensor"]["doorStatus"] = message;
    doc["sensor"]["bootCount"] = bootCount;
    doc["sensor"]["timeAwake"] = millis() - currentMillis;

    // Configuring LoRa
    LoRa.setPins(SS, RST, DI0);

    // Making sure we only see our packets
    LoRa.setSyncWord(SYNCWORD);

    if (!LoRa.begin(BAND))
    { // initialize ratio at 915 MHz
        Serial.println(ERROR);
        while (true)
            ; // if failed, do nothing
    }

    if (IS_DEBUG)
        serializeJson(doc, Serial);

    // initialize digital pin LED_BUILTIN as an output ONLY
    // when we're using it since OUTPUT mode consumes power
    // https://www.youtube.com/watch?v=kUHEHTev3UE
    if (LED_ENABLED)
    {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
    }
    
    // xmit our mailbox alert and then shut the radio off
    LoRa.beginPacket();
    serializeJson(doc, LoRa);
    LoRa.endPacket();
    LoRa.sleep();

    // Turn the LED off and reset the pin mode
    if (LED_ENABLED)
    {
        digitalWrite(LED_BUILTIN, LOW);
        pinMode(LED_BUILTIN, INPUT);
    }
}

/* Esp32 deep sleep function call and save state variables */
void esp32_sleep()
{
    // Store our vars in rtc data attrs
    time_awake_millis = time_awake_millis + (millis() - currentMillis);
    // If we're dealing with a stuck door we don't want the current input pin reading.
    if (doorState == GPIO_DOOR_STUCK_STATE)
    {
        last_doorState = GPIO_DOOR_STUCK_STATE; // because we use -1 in the case of a stuck door, we manually set it again

        // check one last time to see if closed?!
        if (digitalRead(doorSensorPIN) == GPIO_DOOR_CLOSED_STATE)
            {
                // update the state
                doorState = digitalRead(doorSensorPIN);
                
                // fire off a message
                xmit(doorState, humanizedStatus(doorState));
                
                // update our last known state
                last_doorState = doorState;

                //reset the stuckboot counter each time we get a clean GPIO wakeup
                stuckbootCount = 0;
            }
    }
    else 
    {
        last_doorState = digitalRead(doorSensorPIN); //store the last door state generally should be closed
    }
    
    // Humanized diagnostics
    if (IS_DEBUG)
        Serial.printf("\nDoor State: %d \nLast Door State: %d \nAwake: %ld ms", doorState, last_doorState, time_awake_millis);

    // Go to sleep now
    if (IS_DEBUG)
        Serial.println("\nStarting deep sleep now");
    esp_deep_sleep_start(); //Enter deep sleep
}

/* Formatting function to convert millis into h:m:s */
void runtime(unsigned long ms)
{
    unsigned long runMillis = ms;
    unsigned long allSeconds = runMillis / 1000;
    int runHours = allSeconds / 3600;
    int secsRemaining = allSeconds % 3600;
    int runMinutes = secsRemaining / 60;
    int runSeconds = secsRemaining % 60;
    sprintf(total_time_awake, "%02d:%02d:%02d", runHours, runMinutes, runSeconds);
}

/*  Start of the main setup  routine  */
void setup()
{
    Serial.begin(115200);

    // start a timer to see how long we're awake
    currentMillis = millis();

    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    if (IS_DEBUG)
    {
        Serial.println("\n");
        Serial.println(PROJECT_NAME);
        const char compile_date[] = __DATE__ " " __TIME__;
        String BUILD_DATE = String(compile_date).c_str();
        Serial.println("Built: " + BUILD_DATE);
    } else {
        Serial.println("WARN: IS_DEBUG set false");
    }

    // reduce power consumption
    setCpuFrequencyMhz(80);

    // initializing our input as an input
    pinMode(GPIO_INPUT_IO_TRIGGER, INPUT);

    // Disable WiFi and Bluetooth
    if (IS_DEBUG)
        Serial.print("\nDisabling WiFi");
    esp_wifi_deinit();
    if (IS_DEBUG)
        Serial.println(" - Done!");

    if (IS_DEBUG)
        Serial.print("Disabling Bluetooth");
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_mem_release(ESP_BT_MODE_BTDM);
    if (IS_DEBUG)
        Serial.println(" - Done!");

    // What's our current door state - open or closed?
    doorState = digitalRead(doorSensorPIN);

    ++bootCount;
    if (IS_DEBUG)
    {
        Serial.println("Boot Counter  : " + String(bootCount));
        Serial.println("Boot Stuck Ctr: " + String(stuckbootCount));
        Serial.println("Last doorState: " + String(last_doorState));
        Serial.println("Door State    : " + String(doorState));
    }

    if (last_doorState == GPIO_DOOR_STUCK_STATE && doorState == GPIO_DOOR_OPEN_STATE) // is the door stuck open triggering Interrupt
    {
        stuckbootCount++;

        if (stuckbootCount > MAX_STUCK_BOOT_COUNT) //door is still stuck open we need to sleep on a timer
        {
            if (IS_DEBUG)
            {
                Serial.println("\nMax stuck open count door reached");
                Serial.printf("\nPutting ESP into a TIMER WAKEUP of %d secs. \n", (TIMER_SLEEP_MICROSECS / 1000000));
            }
            esp_sleep_enable_timer_wakeup(TIMER_SLEEP_MICROSECS); //lets go to TIMER Interrupt sleep instead of GPIO wakeup
            esp32_sleep();
        }
        else
        {
            delay(5000); //pause maybe stuck lid will close down

            // Wake up when the circuit opens
            esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, GPIO_DOOR_OPEN_STATE); 

            // making sure we persist the stuck door state correctly
            doorState = GPIO_DOOR_STUCK_STATE;

            // sleep
            esp32_sleep();
        }
    }

    //was the door stuck open but now is closed.. re-set the door status send a message to indicate closed
    if (stuckbootCount > MAX_STUCK_BOOT_COUNT || (last_doorState == GPIO_DOOR_STUCK_STATE && doorState == GPIO_DOOR_CLOSED_STATE))
    {
        if (IS_DEBUG)
            Serial.println("it was stuck open but now we're good");
        xmit(doorState, humanizedStatus(doorState));
        stuckbootCount = 0; //reset the stuckboot counter each time we get a clean GPIO wakeup
    }

    //Wake up when it goes high - may be inverted for your reed door sensors
    esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, GPIO_DOOR_OPEN_STATE); // Wake the board when the input goes LOW (contact broken)

    if (doorState != GPIO_DOOR_CLOSED_STATE) //is the door NOT closed?
    {
        if (IS_DEBUG)
            Serial.print("\n");

        // Now send the messages
        xmit(doorState, humanizedStatus(doorState));

        long n = 0;
        while (doorState == GPIO_DOOR_OPEN_STATE) // while the door open, do stuff
        {
            n++;
            // Keep checking the door state
            doorState = digitalRead(doorSensorPIN);

            // print out the state of the button:
            // if (IS_DEBUG)
                // Serial.print(".");

            long elapsed_time = (millis() - start_time);

            if (n % 80 == 0)                                     // handle line feed
                if (IS_DEBUG)
                {
                   // Serial.printf(" - %ld ms \n", elapsed_time); // print the time in every 40 cycles
                   // delay(50);                                   // delay in between reads for stability
                }

            // If the door is still OPEN after MAX_OPENDOOR_TIME time, 
            // assume its stuck open and just send a message
            if (elapsed_time > MAX_OPENDOOR_TIME)
            {
                if (IS_DEBUG)
                    Serial.printf("\nDoor STUCK OPEN for %ld ms > %d ms .. ending loop.\n", elapsed_time, MAX_OPENDOOR_TIME);
                doorState = GPIO_DOOR_STUCK_STATE; // if this is stuck open, override the state so it doesn't report a false close later
                xmit(doorState, humanizedStatus(doorState));

                break;
            }
        }

        //calculate total_time_awake - how long the esp32 board was running
        runtime(millis() - currentMillis + time_awake_millis);
        if (IS_DEBUG)
            Serial.println("\nESP32 total awake time: " + String(total_time_awake));

        // If the door becomes closed, send that update
        if (doorState == GPIO_DOOR_CLOSED_STATE)
            xmit(doorState, humanizedStatus(doorState));
    }
    else if (IS_DEBUG)
        Serial.println("\nDoor state unchanged; nothing to report");

    //Go to sleep now
    esp32_sleep();

} //end of setup

void loop() 
{
    // unused
}
