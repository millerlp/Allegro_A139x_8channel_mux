/**
 * @file Allegro_A139x_Hall_effect.cpp
 * @copyright 2017-2022 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Implements the Allegro_A139x class
 */

#include "Allegro_A139x.h"


/* 
* TODO: try modifying this to instead have the object produce up to 8 outputs variables
*   and just do all the mux switching within the single object
*   In this case the user could send the number of channels to sample, or an array of
*   channels to sample, and then create separate ModularSensor Variable* types for 
*   each of those channels. You can still have unique uuids for each of the sensors
*   in that case. 
*/

// The constructor 
// For a single A139x Hall effect sensor connected to the Mayfly
// AllegroA139x::AllegroA139x(int8_t powerPin, int8_t dataPin,
//                                    uint8_t measurementsToAverage)
//     : Sensor("Allegro A139x", ALLEGROA139X_NUM_VARIABLES,
//              ALLEGROA139X_WARM_UP_TIME_MS, ALLEGROA139X_STABILIZATION_TIME_MS,
//              ALLEGROA139X_MEASUREMENT_TIME_MS, powerPin, dataPin,
//              measurementsToAverage) {}


AllegroA139x::AllegroA139x(PCA9557 gpio8, PCA9536 gpio4,
                    int8_t powerPin, int8_t dataPin,
                    uint8_t measurementsToAverage
                    ) 
    : Sensor("Allegro A139x", ALLEGROA139X_NUM_VARIABLES,
             ALLEGROA139X_WARM_UP_TIME_MS, ALLEGROA139X_STABILIZATION_TIME_MS,
             ALLEGROA139X_MEASUREMENT_TIME_MS, powerPin, dataPin,
             measurementsToAverage),
             _pca9557(gpio8),           
             _pca9536(gpio4) {}                     
             
AllegroA139x::~AllegroA139x() {}

// LPM: Modeled on AOSongDHT.cpp setup
// The question is whether is an efficient way to accomplish
// this, because it's potentially going to be used to create 8 
// redundant _pca9557 objects if you have 8 Hall sensor objects
// It may be more sensible to figure out how to pass a pointer to
// an existing PCA9557 object created outside the Hall sensor library
// and then interact with that via the _muxChannel variable
bool AllegroA139x::setup(void) {
    // Set up the PCA9557 multiplexer to output signals, and set all outputs LOW initially
    _pca9557.setPolarity(IO_NON_INVERTED);
    _pca9557.setMode(IO_OUTPUT);
    _pca9557.setState(IO_LOW); // This should SLEEP all attached AllegroA139x sensors
    
    // Set up the PCA9536 multiplexer
    if( _pca9536.begin() == false){
        Serial.println("PCA9536 not detected");
    }
    // Set all 4 pins on the PCA9536 as outputs - these will control the TMUX1208 multiplexer channel selector
    for (int i = 0; i < 4; i++)
    {
        // pinMode can be used to set an I/O as OUTPUT or INPUT
        _pca9536.pinMode(i, OUTPUT);
    }


    return Sensor::setup();  // this will set pin modes and the setup status bit
}


void AllegroA139x::disableTMUX1208(PCA9536 mux){
    mux.write(3, LOW); // Pulls the X3 pin low on the PCA9536 low on the attached TMUX1208 ENable pin
}


bool AllegroA139x::addSingleMeasurementResult(void) {

// TODO: LPM: explore if having a startSingleMeasurement() function
// in this library would be useful. 

    // Initialize values for each sensor channel
    int32_t sensor_adc = -9999 ;
    // Initialize an array to hold the 8 channel values
    int32_t hallVals [8] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999};

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
        
        for (int i = 0; i<=7; i++){
            // Set the appropriate multiplexer channels
            // Wake the A139x hall sensor by pulling the appropriate channel's pin PCA9557 high
            _pca9557.setState((PCA9557_pin_t)i, IO_HIGH); 
            // Specify TMUX1208 channel via PCA9536
            setPCA9536channel(i, _pca9536);               
            delayMicroseconds(60); 
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
                sensor_adc = -9999;
            }
            MS_DBG(F(" Channel: "), i);
            MS_DBG(F("  Counts:"), sensor_adc);

            // Write this channel's count value into the hallVals array
            hallVals[i] = sensor_adc;

            // Set the PCA9557 multiplexer to turn off (sleep) the Hall effect sensor
            _pca9557.setState((PCA9557_pin_t)i, IO_LOW);
        }
    } else {
        MS_DBG(getSensorNameAndLocation(), F("is not currently measuring!"));
    }
    // Pass the uint32_t sensor_adc value to be verified and added to the 
    // variable array that will be averaged
    // verifyAndAddMeasurementResult(ALLEGROA139X_COUNTS_VAR_NUM, sensor_adc);

    // Pass the values stored in hallVals to the appropriate locations 
    // (verify function lives in SensorBase.cpp)
    verifyAndAddMeasurementResult(HALL0_VAR_NUM, hallVals[0]);
    verifyAndAddMeasurementResult(HALL1_VAR_NUM, hallVals[1]);
    verifyAndAddMeasurementResult(HALL2_VAR_NUM, hallVals[2]);
    verifyAndAddMeasurementResult(HALL3_VAR_NUM, hallVals[3]);
    verifyAndAddMeasurementResult(HALL4_VAR_NUM, hallVals[4]);
    verifyAndAddMeasurementResult(HALL5_VAR_NUM, hallVals[5]);
    verifyAndAddMeasurementResult(HALL6_VAR_NUM, hallVals[6]);
    verifyAndAddMeasurementResult(HALL7_VAR_NUM, hallVals[7]);
    
    // Turn the analog multiplexer back off after making readings, using PCA9536
    disableTMUX1208(_pca9536);

    // Unset the time stamp for the beginning of this measurement
    _millisMeasurementRequested = 0;
    // Unset the status bits for a measurement request (bits 5 & 6)
    _sensorStatus &= 0b10011111;

    return true;
}

void AllegroA139x::setPCA9536channel(uint8_t channel, PCA9536 mux) {
    if (channel == 0) {
        mux.write(0, LOW);
        mux.write(1, LOW);
        mux.write(2, LOW);
        mux.write(3, HIGH); // Pull high to ENable the attached TMUX1208 multiplexer
    } else if (channel == 1) {
        mux.write(0, HIGH);
        mux.write(1, LOW);
        mux.write(2, LOW);
        mux.write(3, HIGH);
    } else if (channel == 2) {
        mux.write(0, LOW);
        mux.write(1, HIGH);
        mux.write(2, LOW);
        mux.write(3, HIGH);
    } else if (channel == 3) {
        mux.write(0, HIGH);
        mux.write(1, HIGH);
        mux.write(2, LOW);
        mux.write(3, HIGH);
    } else if (channel == 4) {
        mux.write(0, LOW);
        mux.write(1, LOW);
        mux.write(2, HIGH);
        mux.write(3, HIGH);        
    } else if (channel == 5) {
        mux.write(0, HIGH);
        mux.write(1, LOW);
        mux.write(2, HIGH);
        mux.write(3, HIGH);        
    } else if (channel == 6) {
        mux.write(0, LOW);
        mux.write(1, HIGH);
        mux.write(2, HIGH);
        mux.write(3, HIGH);        
    } else if (channel == 7) {
        mux.write(0, HIGH);
        mux.write(1, HIGH);
        mux.write(2, HIGH);
        mux.write(3, HIGH);        
    }
}