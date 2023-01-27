// This is designed to work with a EnviroDIY Mayfly v1.1 and adapter board
// carrying a PCA9557 I2C port expander, TMUX1208 analog multiplexer, and 
// hook ups for 8 Allegro A1395 (or A1393 or A1391) hall effect sensors

#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>

// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "mayfly_adapter_hall_mux_test.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "Mayfly002";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = -8;  // Pacific Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!


const int32_t serialBaud = 57600;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep

const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v1.1";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */

// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include "sensors/MaximDS3231.h"

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);
/** End [ds3231] */

// ==========================================================================
// PCA9557 8-channel port expander, I2C controlled
//
//  This is hooked to the SLEEP lines of 8 Hall effect transducers on the 
//  Mayfly adapter board RevA. Pulling a pin connected to a SLEEP pin HIGH
//  will wake the sensor, and pulling it low will put the sensor to sleep.
//  =========================================================================

// This PCA9557 should now be created as a private object within the
// Allegro_A139x.h library. Unless I do the version of the library
// that takes gpio8 and gpio4 as pointers to the 
// AllegroA139x objects
#include <PCA9557.h>  // https://github.com/millerlp/PCA9557

PCA9557 gpio8;

// ===========================================================================
// PCA9536 4-channel port expander
// ===========================================================================
/* The device's fixed I2C address is be 0x41 
    PC9536 has 4 pins that can be set as outputs. This library addresses each
    of them separately, so you sent a pin number and a value (HIGH or LOW)
    On the Mayfly adapter, this chip is attached to the address lines of the
    TMUX1208 analog multiplexer that will read the 8 Hall effect sensors

    ------------------------------
    PC9536 Pin      TMUX1208 Pin
        0               A0
        1               A1
        2               A2
        3               EN - active high; pulling low disables TMUX1208

    ---------------------------------------
    Truth table for TMUX1208 - Pulling EN low disables all channels
    A2      A1      A0      Active channel
    0       0       0           S1
    0       0       1           S2
    0       1       0           S3
    0       1       1           S4
    1       0       0           S5
    1       0       1           S6
    1       1       0           S7
    1       1       1           S8

*/
#include <SparkFun_PCA9536_Arduino_Library.h>

PCA9536 gpio4;


// ==========================================================================
//  Allegro A139x Hall effect sensor
//
// ==========================================================================
/** Start [allegroa139x] 
 *  The Allegro A1391, A1393, A1395 Hall effect sensors put out a ratiometric
 *  voltage signal that can be read by the analog-to-digital convertor on a 
 *  Mayfly board or other microcontroller. When no magnetic field is present 
 *  the signal will sit at mid-scale (around 512 on a 10-bit 0-1023 ADC scale)
 *  and when a magnet is nearby the signal will move towards 0 or 1023 depending
 *  on the polarity of the magnet. For this example, we just return the raw
 *  ADC value (0-1023). 

*/
#include <Allegro_A139x_8channel_mux.h> // https://github.com/millerlp/Allegro_A139x_8channel_mux 

// NOTE: Use -1 for any pins that don't apply or aren't being used.
// The Allegro A139x sensors have a SLEEP pin, which wakes the sensor when 
// pulled high, and puts it to sleep when pulled low. You may choose to connect
// the sensor's Vcc pin to a constant 3V3 power source, and then connect the 
// SLEEP pin to one of the digital pins on the Mayfly to wake the sensor.
const int8_t  hallPower      = -1;    // Power pin (or put the SLEEP pin number here)
const int8_t  hallData       = A0;    // Analog 0 pin
const uint8_t hallNumberReadings = 1;

// LPM: Version where I pass the PCA9557 and PCA9536
// objects - Needs testing
// Form of AllegroA139x hall(gpio8, gpio4, hallPower, hallData, hallNumberReadings);
AllegroA139x hallMUX(gpio8, gpio4, hallPower, hallData, hallNumberReadings);
       
/** End [allegroa139x] */



// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
Variable* variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard),
    new ProcessorStats_FreeRam(&mcuBoard),
    new ProcessorStats_Battery(&mcuBoard), 
    new MaximDS3231_Temp(&ds3231),
    new Hall0_Count(&hallMUX, ""),
    new Hall1_Count(&hallMUX, ""),
    new Hall2_Count(&hallMUX, ""),
    new Hall3_Count(&hallMUX, ""),
    new Hall4_Count(&hallMUX, ""),
    new Hall5_Count(&hallMUX, ""),
    new Hall6_Count(&hallMUX, ""),
    new Hall7_Count(&hallMUX, "")
    // Additional sensor variables can be added here, by copying the syntax
    //   for creating the variable pointer (FORM1) from the
    //   `menu_a_la_carte.ino` example
    // The example code snippets in the wiki are primarily FORM2.
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray;
/** End [variable_arrays] */

// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a logger instance
Logger dataLogger;
/** End [loggers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}



// End of working functions
//===========================================================================

void setup() {
  // Wait for USB connection to be established by PC
    // NOTE:  Only use this when debugging - if not connected to a PC, this
    // could prevent the script from starting
    #if defined SERIAL_PORT_USBVIRTUAL
        while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)) {
            // wait
        }
    #endif

    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Set information pins
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the variable array[s], logger[s], and publisher[s]
    varArray.begin(variableCount, variableList);
    dataLogger.begin(LoggerID, loggingInterval, &varArray);

    // Set up the hall effect multiplexers (PCA9557 and PCA9536)
    hallMUX.setup();
    

    // Set up the sensors
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
    

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    dataLogger.createLogFile(true);  // true = write a new header

    // Call the processor sleep
    dataLogger.systemSleep();

}

void loop() {
    // This will loop, logging data at the interval set by loggingInterval above
    dataLogger.logData();


}
  
