/**
 * Based off of EverlightALSPT19.h, modified by Luke Miller
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
 * @section sensor_alspt19_notes Quick Notes
 * - A simple analog current sensor
 * - Requires a 2.5 - 5.5V power source
 *
 * @section sensor_alspt19_datasheet Sensor Datasheet
 * [Datasheet](https://github.com/EnviroDIY/ModularSensors/wiki/Sensor-Datasheets/Everlight-ALS-PT19.pdf)
 *
 * @section sensor_alspt19_ctor Sensor Constructors
 * {{ @ref EverlightALSPT19::EverlightALSPT19(uint8_t) }}
 * {{ @ref EverlightALSPT19::EverlightALSPT19(int8_t, int8_t, float, float, uint8_t) }}
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
/// @brief The power pin for the A139x 
#define MAYFLY_ALLEGROA139X_POWER_PIN -1
/// @brief The data pin for the A139x on the EnviroDIY Mayfly v1.x
#define MAYFLY_ALLEGROA139X_DATA_PIN A3
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
#endif  // ALLEBROA139X_ADC_RESOLUTION
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
 * @anchor sensor_allegroA139x_voltage
 * @name Voltage
 * The voltage variable from an Allegro A139x Hall effect sensor
 * - Range is dependent on supply voltage 
 *
 * {{ @ref AllegroA139x_Voltage::AllegroA139x_Voltage }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; voltage should have 0
 *
 * The true resolution depends on the ADC, the supply voltage, and the loading
 * resistor, but for simplicity we will use 3, which is an appropriate value for
 * the Mayfly.
 */
#define ALLEGROA139X_VOLTAGE_RESOLUTION 3
/// @brief Sensor variable number; current is stored in sensorValues[0].
#define ALLEGROA139X_VOLTAGE_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "electricCurrent"
#define ALLEGROA139X_VOLTAGE_VAR_NAME "voltage"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "microampere"
#define ALLEGROA139X_VOLTAGE_UNIT_NAME "volt"
/// @brief Default variable short code; "ALLEGROA139XVoltage"
#define ALLEGROA139X_VOLTAGE_DEFAULT_CODE "ALLEGROA139XVoltage"
/**@}*/

// TODO: Restart here adapting to Allegro
/* clang-format off */
/**
 * @brief The Sensor sub-class for the [Everlight ALS-PT19](@ref sensor_alspt19).
 */
/* clang-format on */
class EverlightALSPT19 : public Sensor {
 public:
    /**
     * @brief Construct a new EverlightALSPT19 object with custom supply voltage
     * and loading resistor values.
     *
     * @param powerPin The pin on the mcu controlling power to the AOSong
     * ALS-PT19.  Use -1 if it is continuously powered.
     * - The ALS-PT19 requires a 2.5 - 5.5V power source
     * @param dataPin The processor ADC port pin to read the voltage from the EC
     * probe.  Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A1.
     * @param supplyVoltage The power supply voltage (in volts) of the ALS-PT19.
     * @param loadResistor The size of the loading resistor, in kilaohms (kΩ).
     * @param measurementsToAverage The number of measurements to take and
     * average before giving a "final" result from the sensor; optional with a
     * default value of 10.
     */
    EverlightALSPT19(int8_t powerPin, int8_t dataPin, float supplyVoltage,
                     float loadResistor, uint8_t measurementsToAverage = 10);
    /**
     * @brief Construct a new EverlightALSPT19 object with pins and resistors
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
    explicit EverlightALSPT19(uint8_t measurementsToAverage = 10);
    /**
     * @brief Destroy the EverlightALSPT19 object - no action needed.
     */
    ~EverlightALSPT19();

    /**
     * @copydoc Sensor::addSingleMeasurementResult()
     */
    bool addSingleMeasurementResult(void) override;

 private:
    /**
     * @brief The power supply voltage
     */
    float _supplyVoltage;
    /**
     * @brief The loading resistance
     */
    float _loadResistor;
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [relative current output](@ref sensor_alspt19_voltage) from an
 * [Everlight ALS-PT19](@ref sensor_alspt19).
 */
/* clang-format on */
class EverlightALSPT19_Voltage : public Variable {
 public:
    /**
     * @brief Construct a new EverlightALSPT19_Voltage object.
     *
     * @param parentSense The parent EverlightALSPT19 providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "ALSPT19Current".
     */
    explicit EverlightALSPT19_Voltage(
        EverlightALSPT19* parentSense, const char* uuid = "",
        const char* varCode = ALSPT19_VOLTAGE_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ALSPT19_VOLTAGE_VAR_NUM,
                   (uint8_t)ALSPT19_VOLTAGE_RESOLUTION,
                   ALSPT19_VOLTAGE_VAR_NAME, ALSPT19_VOLTAGE_UNIT_NAME, varCode,
                   uuid) {}
    /**
     * @brief Construct a new EverlightALSPT19_Voltage object.
     *
     * @note This must be tied with a parent EverlightALSPT19 before it can be
     * used.
     */
    EverlightALSPT19_Voltage()
        : Variable((const uint8_t)ALSPT19_VOLTAGE_VAR_NUM,
                   (uint8_t)ALSPT19_VOLTAGE_RESOLUTION,
                   ALSPT19_VOLTAGE_VAR_NAME, ALSPT19_VOLTAGE_UNIT_NAME,
                   ALSPT19_VOLTAGE_DEFAULT_CODE) {}
    /**
     * @brief Destroy the EverlightALSPT19_Voltage object - no action needed.
     */
    ~EverlightALSPT19_Voltage() {}
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [relative current output](@ref sensor_alspt19_current) from an
 * [Everlight ALS-PT19](@ref sensor_alspt19).
 */
/* clang-format on */
class EverlightALSPT19_Current : public Variable {
 public:
    /**
     * @brief Construct a new EverlightALSPT19_Current object.
     *
     * @param parentSense The parent EverlightALSPT19 providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "ALSPT19Current".
     */
    explicit EverlightALSPT19_Current(
        EverlightALSPT19* parentSense, const char* uuid = "",
        const char* varCode = ALSPT19_CURRENT_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ALSPT19_CURRENT_VAR_NUM,
                   (uint8_t)ALSPT19_CURRENT_RESOLUTION,
                   ALSPT19_CURRENT_VAR_NAME, ALSPT19_CURRENT_UNIT_NAME, varCode,
                   uuid) {}
    /**
     * @brief Construct a new EverlightALSPT19_Current object.
     *
     * @note This must be tied with a parent EverlightALSPT19 before it can be
     * used.
     */
    EverlightALSPT19_Current()
        : Variable((const uint8_t)ALSPT19_CURRENT_VAR_NUM,
                   (uint8_t)ALSPT19_CURRENT_RESOLUTION,
                   ALSPT19_CURRENT_VAR_NAME, ALSPT19_CURRENT_UNIT_NAME,
                   ALSPT19_CURRENT_DEFAULT_CODE) {}
    /**
     * @brief Destroy the EverlightALSPT19_Current object - no action needed.
     */
    ~EverlightALSPT19_Current() {}
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [calculated illuminance output](@ref sensor_alspt19_illuminance) from an
 * [Everlight ALS-PT19](@ref sensor_alspt19).
 */
/* clang-format on */
class EverlightALSPT19_Illuminance : public Variable {
 public:
    /**
     * @brief Construct a new EverlightALSPT19_Illuminance object.
     *
     * @param parentSense The parent EverlightALSPT19 providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "ALSPT19Lux".
     */
    explicit EverlightALSPT19_Illuminance(
        EverlightALSPT19* parentSense, const char* uuid = "",
        const char* varCode = ALSPT19_ILLUMINANCE_DEFAULT_CODE)
        : Variable(parentSense, (const uint8_t)ALSPT19_ILLUMINANCE_VAR_NUM,
                   (uint8_t)ALSPT19_ILLUMINANCE_RESOLUTION,
                   ALSPT19_ILLUMINANCE_VAR_NAME, ALSPT19_ILLUMINANCE_UNIT_NAME,
                   varCode, uuid) {}
    /**
     * @brief Construct a new EverlightALSPT19_Illuminance object.
     *
     * @note This must be tied with a parent EverlightALSPT19 before it can be
     * used.
     */
    EverlightALSPT19_Illuminance()
        : Variable((const uint8_t)ALSPT19_ILLUMINANCE_VAR_NUM,
                   (uint8_t)ALSPT19_ILLUMINANCE_RESOLUTION,
                   ALSPT19_ILLUMINANCE_VAR_NAME, ALSPT19_ILLUMINANCE_UNIT_NAME,
                   ALSPT19_ILLUMINANCE_DEFAULT_CODE) {}
    /**
     * @brief Destroy the EverlightALSPT19_Illuminance object - no action
     * needed.
     */
    ~EverlightALSPT19_Illuminance() {}
};
/**@}*/
#endif  // SRC_SENSORS_EVERLIGHTALSPT19_H_
