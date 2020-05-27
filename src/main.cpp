// ** Sender code **

// show debug output
#define IS_DEBUG true // enable Serial.println statements

// #include <SPI.h> // Only needed for OLED display
#include <LoRa.h>
#include <Wire.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"

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
#define LED_ENABLED true                       // Turn on the on-board LED when mailbox door is open?
#define MAX_OPENDOOR_TIME 30000                // default 30s in milliseconds how long to wait while door is open to consider it stuck open
#define MAX_STUCK_BOOT_COUNT 5                 // If the door is stuck for more than x times let's switch to timer interrupt to save battery
#define TIMER_SLEEP_MICROSECS 1800 * 1000000   // when on timer interrupt how long to sleep in seconds * microseconds

// sensor/door_status messages for open closed and stuck
#define message_door_open "DOOR_OPEN"
#define message_door_closed "DOOR_CLOSED"
#define message_door_stuck "DOOR_STUCK_OPEN"

// Define the door Sensor PIN and initial state (Depends on your reed sensor N/C or N/O )
gpio_num_t doorSensorPIN = GPIO_NUM_34;             // GPIO for the contact closure
gpio_num_t GPIO_INPUT_IO_TRIGGER = doorSensorPIN;
int GPIO_DOOR_CLOSED_STATE = HIGH;                  // Default state when the reed and magnet are next to each other (depends on reed switch)
int GPIO_DOOR_OPEN_STATE = !GPIO_DOOR_CLOSED_STATE; // Open state is oposite of closed

// RTC Memory attribute is retained across resets
RTC_DATA_ATTR long bootCount = 0;
RTC_DATA_ATTR long stuckbootCount = 0;    // if the door is stuck open increment this counter
RTC_DATA_ATTR long last_doorState = -99;  // will store last door state to detect stuck open
RTC_DATA_ATTR long time_awake_millis = 0; // total awake time in milliseconds - useful for tracking how long battery lasts

long currentMillis;        // timer start of awake
int doorState = LOW;       // maintains the door sensor state usually LOW= 0 or 1
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

/* Esp32 deep sleep function call and save state variables */
void esp32_sleep()
{
   // Store our vars in rtc data attrs
    time_awake_millis = time_awake_millis + (millis() - currentMillis);
    last_doorState = digitalRead(doorSensorPIN); //store the last door state generally should be closed
    
    // Humanized diagnostics
    if (IS_DEBUG)
        Serial.printf("\nDoor State: %d \nLast Door State: %d \nAwake: %ld ms", doorState, last_doorState, time_awake_millis);

    // Go to sleep now
    if (IS_DEBUG)
        Serial.println("\nStarting deep sleep now");
    esp_deep_sleep_start(); //Enter deep sleep
}

void xmit(String msgType, String message)
{
    if (!LoRa.begin(BAND))
    {
        Serial.println(ERROR);
        delay(100);
    }

    String payload = msgType + "|" + message;
    if (IS_DEBUG)
        Serial.println("payload: " + payload);

    // xmit our mailbox alert and then shut the radio off
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();
    LoRa.sleep();
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

    // Configuring LoRa
    LoRa.setPins(SS, RST, DI0);

    if (IS_DEBUG)
    {
        Serial.println("\n");
        Serial.println(PROJECT_NAME);
        Serial.print("Built: ");
        Serial.print(__DATE__);
        Serial.print(" @ ");
        Serial.println(__TIME__);
    } else {
        Serial.println("WARN: IS_DEBUG set false");
    }

    // start a timer to see how long we're awake
    currentMillis = millis();

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    // initializing our input as an input
    pinMode(GPIO_INPUT_IO_TRIGGER, INPUT);

    // Making sure we only see our packets
    LoRa.setSyncWord(SYNCWORD);

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

    //!< Keep power domain enabled in deep sleep, if it is needed by one of the wakeup options. Otherwise power it down.
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);

    ++bootCount;
    if (IS_DEBUG)
    {
        Serial.println("Boot Counter  : " + String(bootCount));
        Serial.println("Boot Stuck Ctr: " + String(stuckbootCount));
        Serial.println("Last Door     : " + String(last_doorState));
    
        // Serial.println("\nDoor state : " + String(doorState));
        // Serial.println("Closed door = " + String(GPIO_DOOR_CLOSED_STATE));
        // Serial.println("Open door   = " + String(GPIO_DOOR_OPEN_STATE));
    }

    if (last_doorState == doorState) // is the door stuck open triggering Interrupt
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

            // Wake up when it goes high - may be inverted for your reed door sensors
            esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, GPIO_DOOR_OPEN_STATE); //1 = High, 0 = Low  wake door OPEN (magnet away sensor)
            esp32_sleep();
        }
    }

    //was the door stuck open but now is closed.. re-set the door status send a message to indicate closed
    if (stuckbootCount > MAX_STUCK_BOOT_COUNT)
    {
        xmit("status", String(doorState));
        xmit("string", message_door_closed);
        // if (connect_WIFI_MQTT())
        // {
        //     client.publish("/sensor/door", String(doorState).c_str(), false);  //false means don't retain messages
        //     client.publish("/sensor/door_status", message_door_closed, false); //text version of door
        // }

        stuckbootCount = 0; //reset the stuckboot counter each time we get a clean GPIO wakeup
    }

    //Wake up when it goes high - may be inverted for your reed door sensors
    esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, GPIO_DOOR_OPEN_STATE); //1 = High, 0 = Low  wake door OPEN (magnet away sensor)

    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    if (IS_DEBUG)
    {
        Serial.println("Door state currently: " + String(doorState));
        Serial.println("Closed state: " + String(GPIO_DOOR_CLOSED_STATE));
    }

    if (doorState != GPIO_DOOR_CLOSED_STATE) //is the door NOT closed?
    {
        if (LED_ENABLED)
            digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on

        // Now send the messages
        xmit("status", String(doorState));
        xmit("string", message_door_open);
        xmit("boot", String(bootCount));

        long n = 0;                               //loop counter while door is opened
        while (doorState == GPIO_DOOR_OPEN_STATE) //while where open
        {
            n++;
            // client.loop(); //call regularly to keep the connection to mqtt broker open

            // Keep reading the door state
            doorState = digitalRead(doorSensorPIN);

            // print out the state of the button:
            if (IS_DEBUG)
                Serial.print(".");

            long elapsed_time = (millis() - start_time);

            if (n % 40 == 0)                                     // handle line feed
                if (IS_DEBUG)
                {
                    Serial.printf(" - %ld ms \n", elapsed_time); // print the time in every 40 cyclels
                    delay(50);                                   // delay in between reads for stability
                }

            // If the door is still OPEN after MAX_OPENDOOR_TIME time, 
            // assume its stuck open and just send a message
            if (elapsed_time > MAX_OPENDOOR_TIME)
            {
                if (IS_DEBUG)
                    Serial.printf("\nDoor STUCK OPEN for %ld ms > %d ms .. ending loop. ", elapsed_time, MAX_OPENDOOR_TIME);
                xmit("status", String(doorState));
                xmit("string", message_door_stuck); 

                break;
            }
        }

        //calculate total_time_awake - how long the esp32 board was running
        runtime(millis() - currentMillis + time_awake_millis);
        Serial.println("\nESP32 total awake time: " + String(total_time_awake));

        if (doorState == LOW)                                                  //only if the door is really closed , check in case we fell through while loop
            xmit("string", message_door_closed);                               //text version of /sensor/door
        
        xmit("totalAwake", String(total_time_awake));
        
        if (LED_ENABLED)
            digitalWrite(LED_BUILTIN, LOW); // :: Turn the LED oFF
    }
    else if (IS_DEBUG)
        Serial.println("\nDoor state unchanged; not reporting?");

    //Go to sleep now
    esp32_sleep();

} //end of setup

void loop() 
{
    // unused
}
