/**
 * @file Allegro_A139x_Hall_effect.cpp
 * @copyright 2017-2022 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Implements the Allegro_A139x class
 */

#include "Allegro_A139x.h"


// The constructor 
// For a single A139x Hall effect sensor connected to the Mayfly
AllegroA139x::AllegroA139x(int8_t powerPin, int8_t dataPin,
                                   uint8_t measurementsToAverage)
    : Sensor("Allegro A139x", ALLEGROA139X_NUM_VARIABLES,
             ALLEGROA139X_WARM_UP_TIME_MS, ALLEGROA139X_STABILIZATION_TIME_MS,
             ALLEGROA139X_MEASUREMENT_TIME_MS, powerPin, dataPin,
             measurementsToAverage){}
// Version with multiplexer channel variable included, for 8-channel adapter board             
AllegroA139x::AllegroA139x(int8_t powerPin, int8_t dataPin,
                    uint8_t measurementsToAverage = 4,
                    uint8_t muxChannel)         
    : Sensor("Allegro A139x", ALLEGROA139X_NUM_VARIABLES,
             ALLEGROA139X_WARM_UP_TIME_MS, ALLEGROA139X_STABILIZATION_TIME_MS,
             ALLEGROA139X_MEASUREMENT_TIME_MS, powerPin, dataPin,
             measurementsToAverage),
             _muxChannel(muxChannel){}
// Short-cut version for single sensor on default channel + settings             
AllegroA139x::AllegroA139x(uint8_t measurementsToAverage)
    : Sensor("Allegro A139x", ALLEGROA139X_NUM_VARIABLES,
             ALLEGROA139X_WARM_UP_TIME_MS, ALLEGROA139X_STABILIZATION_TIME_MS,
             ALLEGROA139X_MEASUREMENT_TIME_MS, MAYFLY_ALLEGROA139X_POWER_PIN,
            MAYFLY_ALLEGROA139X_DATA_PIN, measurementsToAverage,
            ALLEGROA139X_INC_CALC_VARIABLES) {}
AllegroA139x::~AllegroA139x() {}


bool AllegroA139x::addSingleMeasurementResult(void) {

// TODO: LPM: work out how to make a version of this function that can
// use the multiplexers when requested, based on the private
// variable _muxChannel being available. This may require the
// use of a #define macro to tell the library whether to run
// multiplexer code or ignore it.
// TODO: LPM: explore if having a startSingleMeasurement() function
// in this library would be useful. 

    // Initialize float variables
    // float volt_val    = -9999;
    // float current_val = -9999;
    // float lux_val     = -9999;
    int32_t sensor_adc = -9999 ;

    // Check a measurement was *successfully* started (status bit 6 set)
    // Only go on to get a result if it was 
    // TODO: Check if this is useful, I think not?
    if (bitRead(_sensorStatus, 6)) {
        // Set the resolution for the processor ADC, only applies to SAMD
        // boards.
#if !defined ARDUINO_ARCH_AVR
        analogReadResolution(ALLEGROA139X_ADC_RESOLUTION);
#endif  // ARDUINO_ARCH_AVR
        // Set the analog reference mode for the voltage measurement.
        // If possible, to get the best results, an external reference should be
        // used.
        analogReference(ALLEGROA139X_ADC_REFERENCE_MODE);
        MS_DBG(getSensorNameAndLocation(), F("is reporting:"));

        // First measure the analog voltage.
        // The return value from analogRead() is IN BITS NOT IN VOLTS!!
        // Take a priming reading.
        // First reading will be low - discard
        analogRead(_dataPin);
        // Take the reading we'll keep
        sensor_adc = analogRead(_dataPin);
        MS_DEEP_DBG("  ADC Bits:", sensor_adc);

        if (0 == sensor_adc) {
            // Prevent underflow, can never be ALLEGROA139X_ADC_RANGE
            sensor_adc = 1;
        }
        // convert bits to volts
        // volt_val = (_supplyVoltage / static_cast<float>(ALSPT19_ADC_MAX)) *
            // static_cast<float>(sensor_adc);
        // convert volts to current
        // resistance is entered in kΩ and we want µA
        // current_val = (volt_val / (_loadResistor * 1000)) * 1e6;
        // convert current to illuminance
        // from sensor datasheet, typical 200µA current for1000 Lux
        // lux_val = current_val * (1000. / 200.);


        MS_DBG(F("  Counts:"), sensor_adc);
        // MS_DBG(F("  Current:"), current_val, F("µA"));
        // MS_DBG(F("  Illuminance:"), lux_val, F("lux"));
    } else {
        MS_DBG(getSensorNameAndLocation(), F("is not currently measuring!"));
    }
    // Pass the uint32_t sensor_adc value to be verified and added to the 
    // variable array that will be averaged
    verifyAndAddMeasurementResult(ALLEGROA139X_COUNTS_VAR_NUM, sensor_adc);
    
    

    // Unset the time stamp for the beginning of this measurement
    _millisMeasurementRequested = 0;
    // Unset the status bits for a measurement request (bits 5 & 6)
    _sensorStatus &= 0b10011111;

    return true;
}
