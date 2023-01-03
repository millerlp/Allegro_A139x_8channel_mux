/**
 * Based off of EverlightALSPT19.h, modified by Luke Miller to read Allegro A139x Hall effect sensors
 * @copyright 2017-2022 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Take readings from Allegro A1395, A1393, A1391 Hall effect sensors
 * via the onboard ADC
 *
 */
/* clang-format off */
/**
 * @defgroup sensor_allegroA139x Allegro A139x 
 * Classes for the Allegro A139x Hall effect sensor.
 *
 * @ingroup the_sensors
 *
 * @tableofcontents
 * @m_footernavigation
 *
 * @section sensor_allegroa139x_notes Quick Notes
 * - A simple analog Hall effect sensor
 * - Requires a 2.5 - 3.5V power source
 *
 * @section sensor_allegroa139x_datasheet Sensor Datasheet
 * [Datasheet]()
 *
 * @section sensor_allegroa139x_ctor Sensor Constructors
 * {{ @ref AllegroA139x::AllegroA139x(uint8_t) }}
 * {{ @ref AllegroA139x::AllegroA139x(int8_t, int8_t, uint8_t) }}
 *
 * @section sensor_alspt19_examples Example Code
 *
 * The ALS-PT19 is used in the @menulink{everlight_alspt19} example
 *
 * @menusnip{everlight_alspt19}
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_ALLEGROA139X_H_
#define SRC_SENSORS_ALLEGROA139X_H_

// Debugging Statement
// #define MS_EVERLIGHTALSPT19_DEBUG

#ifdef MS_ALLEGROA139X_DEBUG
#define MS_DEBUGGING_STD "AllegroA139x"
#endif
#ifdef MS_ALLEGROA139X_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "AllegroA139x"
#endif

// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP
#include "VariableBase.h"
#include "SensorBase.h"

/** @ingroup sensor_allegroA139x */
/**@{*/

// Sensor Specific Defines
/// @brief Sensor::_numReturnedValues; the A139x can report 1 "raw" value
/// (voltage).
#define ALLEGROA139X_NUM_VARIABLES 1
/// @brief Sensor::_incCalcValues; we don't calculate any variables
#define ALLEGROA139X_INC_CALC_VARIABLES 0
/// @brief The power pin for the A139x, -1 if always on 
#define MAYFLY_ALLEGROA139X_POWER_PIN -1
/// @brief The data pin for the A139x on the EnviroDIY Mayfly v1.x
#define MAYFLY_ALLEGROA139X_DATA_PIN A0
/// @brief The supply voltage for the A139x on the EnviroDIY Mayfly v1.x
#define MAYFLY_ALLEGROA139X_SUPPLY_VOLTAGE 3.3


#if !defined ALLEGROA139X_ADC_RESOLUTION
/**
 * @brief Default resolution (in bits) of the voltage measurement
 *
 * The default for all boards is 10, use a build flag to change this, if
 * necessary.
 */
#define ALLEGROA139X_ADC_RESOLUTION 10
#endif  // ALLEGROA139X_ADC_RESOLUTION
/// @brief The maximum possible value of the ADC - one less than the resolution
/// shifted up one bit.
#define ALLEGROA139X_ADC_MAX ((1 << ALLEGROA139X_ADC_RESOLUTION) - 1)
/// @brief The maximum possible range of the ADC - the resolution shifted up one
/// bit.
#define ALLEGROA139X_ADC_RANGE (1 << ALLEGROA139X_ADC_RESOLUTION)

/* clang-format off */
#if !defined ALLEGROA139X_ADC_REFERENCE_MODE
#if defined ARDUINO_ARCH_AVR | defined DOXYGEN
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For an AVR board, this must be one of:
 * - `DEFAULT`: the default built-in analog reference of 5 volts (on 5V Arduino
 * boards) or 3.3 volts (on 3.3V Arduino boards)
 * - `INTERNAL`: a built-in reference, equal to 1.1 volts on the ATmega168 or
 * ATmega328P and 2.56 volts on the ATmega32U4 and ATmega8 (not available on the
 * Arduino Mega)
 * - `INTERNAL1V1`: a built-in 1.1V reference (Arduino Mega only)
 * - `INTERNAL2V56`: a built-in 2.56V reference (Arduino Mega only)
 * - `EXTERNAL`: the voltage applied to the AREF pin (0 to 5V only) is used as the
 * reference.
 *
 * If not set on an AVR board `DEFAULT` is used.
 *
 * For the best accuracy, use an `EXTERNAL` reference with the AREF pin
 * connected to the power supply for the EC sensor.
 */
#define ALLEGROA139X_ADC_REFERENCE_MODE DEFAULT
#endif
#if defined ARDUINO_ARCH_SAMD | defined DOXYGEN
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For a SAMD board, this must be one of:
 * - `AR_DEFAULT`: the default built-in analog reference of 3.3V
 * - `AR_INTERNAL`: a built-in 2.23V reference
 * - `AR_INTERNAL1V0`: a built-in 1.0V reference
 * - `AR_INTERNAL1V65`: a built-in 1.65V reference
 * - `AR_INTERNAL2V23`: a built-in 2.23V reference
 * - `AR_EXTERNAL`: the voltage applied to the AREF pin is used as the reference
 *
 * If not set on an SAMD board `AR_DEFAULT` is used.
 *
 * For the best accuracy, use an `EXTERNAL` reference with the AREF pin
 * connected to the power supply for the EC sensor.
 *
 * @see https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
 */
#define ALLEGROA139X_ADC_REFERENCE_MODE AR_DEFAULT
#endif
#if !defined ALLEGROA139X_ADC_REFERENCE_MODE
#error The processor ADC reference type must be defined!
#endif  // ALLEGROA139X_ADC_REFERENCE_MODE
#endif  // ARDUINO_ARCH_SAMD
/* clang-format on */

/**
 * @anchor sensor_allegroA139x_timing
 * @name Sensor Timing
 * The sensor timing for an Allegro A139x Hall effect sensor
 */
/**@{*/
/// @brief Sensor::_warmUpTime_ms; the A139x should not need more than 60 usec to warm up.
#define ALLEGROA139X_WARM_UP_TIME_MS 1
/// @brief Sensor::_stabilizationTime_ms; the A139x rise time is 60usec
/// (typical).
#define ALLEGROA139X_STABILIZATION_TIME_MS 1
/// @brief Sensor::_measurementTime_ms; essentially 0 time is taken in a
/// reading, just the analog read time
#define ALLEGROA139X_MEASUREMENT_TIME_MS 0
/**@}*/

/**
 * @anchor sensor_allegroA139x_counts
 * @name Counts
 * The Counts variable from an Allegro A139x Hall effect sensor
 * - Range is dependent on ADC resolution (default 10 bits, 0-1023) 
 *
 * {{ @ref AllegroA139x_Counts::AllegroA139x_Counts }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; voltage should have 0
 *
 * The true resolution depends on the ADC, the supply voltage, and the loading
 * resistor, but for simplicity we will use 3, which is an appropriate value for
 * the Mayfly.
 */
#define ALLEGROA139X_COUNTS_RESOLUTION 3
/// @brief Sensor variable number; raw ADC count is stored in sensorValues[0].
#define ALLEGROA139X_COUNTS_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "electricCurrent"
#define ALLEGROA139X_COUNTS_VAR_NAME "counts"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "microampere"
#define ALLEGROA139X_COUNTS_UNIT_NAME "count"
/// @brief Default variable short code; "AllegroA139xCounts"
#define ALLEGROA139X_COUNTS_DEFAULT_CODE "AllegroA139xCounts"
/**@}*/

// 
/* clang-format off */
/**
 * @brief The Sensor sub-class for the AllegroA139x Hall effect sensor.
 */
/* clang-format on */
class AllegroA139x : public Sensor {
 public:
    /**
     * @brief Construct a new AllegroA139x object to read one sensor
     * 
     *
     * @param powerPin The pin on the mcu controlling power to the Allegro A139x
     *   Use -1 if it is continuously powered.
     * - The A139x sensors require a 2.5 - 3.5V power source
     *  The Allegro A139x sensors have a SLEEP pin, which wakes the sensor when
     *  pulled high, and puts it to sleep when pulled low. You may choose to connect
     *  the sensor's Vcc pin to a constant 3V3 power source, and then connect the 
     * SLEEP pin to one of the digital pins on the Mayfly to wake the sensor, and 
     * name that digital pin here as the powerPin.
     * @param dataPin The processor ADC port pin to read the voltage from the
     * sensor.  Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A1.
     * @param measurementsToAverage The number of measurements to take and
     * average before giving a "final" result from the sensor; optional with a
     * default value of 10.
     */
    AllegroA139x(int8_t powerPin, int8_t dataPin,
                    uint8_t measurementsToAverage = 10);

    /**
     * @brief Construct a new AllegroA139x object for use with the 8-channel Mayfly adapter board
     * 
     * This is a constructor for a Hall effect sensor object, when used with an 8-channel
     * adapter board on the Mayfly v1.1. The adapter board uses the Mayfly's PCA9536 4-channel
     * I2C expander to interface with channel-selection pins on a TMUX1208 multiplexer, which 
     * is hooked to the 8 voltage-out pins of 8 A139x Hall effect sensors and can be used
     * to select with sensor's voltage is sent to the dataPin on the Mayfly to be read as
     * an analog voltage. This also uses an 8-channel I2C PCA9557 port expander chip on the 
     * adapter board that is hooked to the 8 SLEEP pins of the 8 A139x Hall effect sensors,
     * and should allow for the designated sensor to be awakened/slept before and after each
     * reading. 
     * 
     *
     * @param powerPin The pin on the mcu controlling power to the Allegro A139x
     *   Use -1 if it is continuously powered or you are controlling the SLEEP
     *   pin via a I2C port expander (PCA9557)
     * @param dataPin The processor ADC port pin to read the voltage from the
     * sensor.  Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A0. The 8-channel adapter board sends data to the A0 pin of the Mayfly
     * by default.
     * @param measurementsToAverage The number of measurements to take and
     * average before giving a "final" result from the sensor; optional with a
     * default value of 10.
     * @param muxChannel The channel (0-7) that this A139x is attached to on the
     * PCA9557 I2C port expander (controlling the SLEEP pin) and on the 
     * TMUX1208 analog (de)multiplexer that will feed the A139x's voltage signal
     * to the dataPin defined above. The TMUX1208 channel is set using the 
     * Mayfly v1.1's onboard PCA9536 I2C port expander that is connected to the
     * TMUX1208's channel selection pins (X0, X1, X2 of PCA9536 connected to A0, 
     * A1, A2 on the TMUX1208). 
     */
   AllegroA139x(int8_t powerPin, int8_t dataPin,
                    uint8_t measurementsToAverage = 10,
                    uint8_t muxChannel);

    /**
     * @brief Construct a new AllegroA139x object with pins 
     * for the EnviroDIY Mayfly 1.x.
     *
     * This is a short-cut constructor to help users of our own board so they
     * can change the number of readings without changing other arguments or
     * enter no arguments at all.
     *
     * @param measurementsToAverage The number of measurements to take and
     * average before giving a "final" result from the sensor; optional with a
     * default value of 10.
     */
    explicit AllegroA139x(uint8_t measurementsToAverage = 10);
    /**
     * @brief Destroy the AllegroA139x object - no action needed.
     */
    ~AllegroA139x();

    /**
     * @copydoc Sensor::addSingleMeasurementResult()
     */
    bool addSingleMeasurementResult(void) override;

 private:
    /**
     * 
     */

};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 */
/* clang-format on */
class AllegroA139x_Counts : public Variable {
 public:
    /**
     * @brief Construct a new AllegroA139x_Counts object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139xCounts".
     */
    explicit AllegroA139x_Counts(
        AllegroA139x* parentSense, const char* uuid = "",
        const char* varCode = ALLEGROA139X_COUNTS_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ALLEGROA139X_COUNTS_VAR_NUM,
                   (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                   ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME, varCode,
                   uuid) {}
    /**
     * @brief Construct a new AllegroA139x_Counts object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be
     * used.
     */
    AllegroA139x_Counts()
        : Variable((const uint8_t)ALLEGROA139X_COUNTS_VAR_NUM,
                   (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                   ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                   ALLEGROA139X_COUNTS_DEFAULT_CODE) {}
    /**
     * @brief Destroy the AllegroA139x_Counts object - no action needed.
     */
    ~AllegroA139x_Counts() {}
};






/**@}*/
#endif  // SRC_SENSORS_ALLEGROA139X_H_
