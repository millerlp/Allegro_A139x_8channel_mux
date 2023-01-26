/**
 * @file Allegro_A139x_8channel_mux.h
 * @copyright 2022-2023 Luke Miller
 * A branch of the EnviroDIY ModularSensors library for Arduino
 * @author Luke Miller <contact@lukemiller.org>
 *
 * @brief Take readings from multiplexed Allegro A1395, A1393, A1391 Hall effect sensors
 *  using PCA9557 to wake each sensor and PCA9536 to route sensor
 * output through a TMUX1208 multiplexer to the onboard ADC.
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
 * - A simple multiplexed analog Hall effect sensor 
 * - Requires a 2.5 - 3.5V power source
 *
 * @section sensor_allegroa139x_datasheet Sensor Datasheet
 * [Datasheet]()
 *
 * @section sensor_allegroa139x_ctor Sensor Constructors
 * 
 * {{ @ref AllegroA139x::AllegroA139x(PCA9557, PCA9536, int8_t, int8_t, uint8_t) }}
 *
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_ALLEGROA139X_8CHANNEL_MUX_H_
#define SRC_SENSORS_ALLEGROA139X_8CHANNEL_MUX_H_

// Debugging Statement

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
#include "PCA9557.h"
#include "SparkFun_PCA9536_Arduino_Library.h"
#include "VariableBase.h"
#include "SensorBase.h"

/** @ingroup sensor_allegroA139x */
/**@{*/

// Sensor Specific Defines
/// @brief Sensor::_numReturnedValues; the multiplexed A139x adapter board can report 8 "raw" value
/// (voltage).
#define ALLEGROA139X_NUM_VARIABLES 8
/// @brief Sensor::_incCalcValues; we don't calculate any variables
#define ALLEGROA139X_INC_CALC_VARIABLES 0
/// @brief The power pin for the A139x, -1 if always on 
#define MAYFLY_ALLEGROA139X_POWER_PIN -1
/// @brief The data pin for the A139x on the EnviroDIY Mayfly v1.x with Hall effect adapter
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

// 
/// @brief Sensor variable number; Hall0 is stored in sensorValues[0].
#define HALL0_VAR_NUM 0
/// @brief Sensor variable number; Hall1 is stored in sensorValues[1].
#define HALL1_VAR_NUM 1
/// @brief Sensor variable number; Hall2 is stored in sensorValues[2].
#define HALL2_VAR_NUM 2
/// @brief Sensor variable number; Hall3 is stored in sensorValues[3].
#define HALL3_VAR_NUM 3
/// @brief Sensor variable number; Hall4 is stored in sensorValues[4].
#define HALL4_VAR_NUM 4
/// @brief Sensor variable number; Hall5 is stored in sensorValues[5].
#define HALL5_VAR_NUM 5
/// @brief Sensor variable number; Hall6 is stored in sensorValues[6].
#define HALL6_VAR_NUM 6
/// @brief Sensor variable number; Hall7 is stored in sensorValues[7].
#define HALL7_VAR_NUM 7

/// @brief Default variable short code;
#define HALL0_DEFAULT_CODE "Hall0counts"
#define HALL1_DEFAULT_CODE "Hall1counts"
#define HALL2_DEFAULT_CODE "Hall2counts"
#define HALL3_DEFAULT_CODE "Hall3counts"
#define HALL4_DEFAULT_CODE "Hall4counts"
#define HALL5_DEFAULT_CODE "Hall5counts"
#define HALL6_DEFAULT_CODE "Hall6counts"
#define HALL7_DEFAULT_CODE "Hall7counts"


/**@}*/

/* clang-format off */
/**
 * @brief The Sensor sub-class for the AllegroA139x Hall effect sensor.
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class AllegroA139x : public Sensor {
 public:
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
     * @param gpio8 The PCA9557 object. The 8 output pins from this device are attached to the
     * SLEEP lines on the AllegroA139x hall effect sensors. 
     * @param gpio4 The PCA9536 object. The 4 pins from this device are attached to the
     * address (channel selection) pins on the TMUX1208 multiplexer and the multiplexer's
     * ENABLE pin. 
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
     * default value of 1.

     */
    // Pass references to PCA9557, PCA9536 objects to run those multiplexers
    AllegroA139x(PCA9557 gpio8, PCA9536 gpio4,
                int8_t powerPin, int8_t dataPin,
                uint8_t measurementsToAverage = 1
                );

    /**
     * @brief Destroy the AllegroA139x object - no action needed.
     */
    ~AllegroA139x();

         /**
     * @brief Activate a channel (0-7) on the TMUX1208 using the PCA9536
     * 
     * The address lines on the TMUX1208 on the Mayfly adapter board are
     * hooked to pins X0,X1,X2 of the Mayfly's onboard PCA9536. This function
     * will set those address lines to activate the chosen channel (0-7)
     */
    void setPCA9536channel(uint8_t channel, PCA9536 mux);

    /**
     * @brief Disable the TMUX1208 multiplexer using PCA9536.
     * 
     * The ENable line on the TMUX1208 on the Mayfly adapter board is
     * hooked to pin X3 of the Mayfly's onboard PCA9536. This function
     * will pull that pin low to disable the TMUX1208.
     */
    void disableTMUX1208(PCA9536 mux);

    /**
     * @copydoc Sensor::setup()
     */
    bool setup(void) override;

    /**
     * @copydoc Sensor::addSingleMeasurementResult()
     */
    bool addSingleMeasurementResult(void) override;
    // The 'override' identifier indicates that this library's
    // version of addSingleMeasurementResult() should be used
    // instead of the Base Sensor class's addSingleMeasurementResult()
    // generic function. 



 private:
  PCA9557 _pca9557;  // Create private version of PCA9557 object
  PCA9536 _pca9536;  // Create private version of PCA9536 object
};

/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall0_Count : public Variable {
  public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
    explicit Hall0_Count(AllegroA139x* parentSense,
                           const char* uuid = "", 
                           const char* varCode = HALL0_DEFAULT_CODE) 
         : Variable(parentSense, (const uint8_t)HALL0_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall0_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall0_Count()
        : Variable((const uint8_t) HALL0_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL0_DEFAULT_CODE) {}

    /**
     * @brief Destroy the Hall0 object - no action needed.
     */
    ~Hall0_Count() {}
      
};

/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall1_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
    explicit Hall1_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL1_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL1_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall1_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall1_Count()
        : Variable((const uint8_t) HALL1_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL1_DEFAULT_CODE) {}                     

    /**
     * @brief Destroy the Hall1 object - no action needed.
     */
    ~Hall1_Count() {}
      
};

/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall2_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
      explicit Hall2_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL2_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL2_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall2_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall2_Count()
        : Variable((const uint8_t) HALL2_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL2_DEFAULT_CODE) {}    
    /**
     * @brief Destroy the Hall2 object - no action needed.
     */
    ~Hall2_Count() {}
      
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall3_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
      explicit Hall3_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL3_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL3_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall3_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall3_Count()
        : Variable((const uint8_t) HALL3_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL3_DEFAULT_CODE) {}    
    /**
     * @brief Destroy the Hall3 object - no action needed.
     */
    ~Hall3_Count() {}
      
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall4_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
      explicit Hall4_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL4_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL4_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall4_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall4_Count()
        : Variable((const uint8_t) HALL4_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL4_DEFAULT_CODE) {}    
    /**
     * @brief Destroy the Hall4 object - no action needed.
     */
    ~Hall4_Count() {}
      
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall5_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
      explicit Hall5_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL5_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL5_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall5_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall5_Count()
        : Variable((const uint8_t) HALL5_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL5_DEFAULT_CODE) {}    
    /**
     * @brief Destroy the Hall5 object - no action needed.
     */
    ~Hall5_Count() {}
      
};


/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall6_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
      explicit Hall6_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL6_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL6_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall6_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall6_Count()
        : Variable((const uint8_t) HALL6_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL6_DEFAULT_CODE) {}    
    /**
     * @brief Destroy the Hall1 object - no action needed.
     */
    ~Hall6_Count() {}
      
};

/* clang-format off */
/**
 * @brief The Variable sub-class used for the
 * [adc output](@ref sensor_allegroa139x_counts) from an
 * [Allegro A139x](@ref sensor_allegroa139x).
 * 
 * @ingroup sensor_allegroA139x
 */
/* clang-format on */
class Hall7_Count : public Variable {
   public: 
     /**
     * @brief Construct a new hall effect sensor object.
     *
     * @param parentSense The parent AllegroA139x providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "AllegroA139x".
     */
      explicit Hall7_Count(AllegroA139x* parentSense,
                           const char* uuid = "",
                           const char* varCode = HALL7_DEFAULT_CODE)
         : Variable(parentSense, (const uint8_t)HALL7_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                     varCode, uuid) {}
     /**
     * @brief Construct a new Hall7_Count object.
     *
     * @note This must be tied with a parent AllegroA139x before it can be used.
     */
    Hall7_Count()
        : Variable((const uint8_t) HALL7_VAR_NUM,
                    (uint8_t)ALLEGROA139X_COUNTS_RESOLUTION,
                    ALLEGROA139X_COUNTS_VAR_NAME, ALLEGROA139X_COUNTS_UNIT_NAME,
                    HALL7_DEFAULT_CODE) {}    
    /**
     * @brief Destroy the Hall1 object - no action needed.
     */
    ~Hall7_Count() {}
      
};






/**@}*/
#endif  // SRC_SENSORS_ALLEGROA139X_8CHANNEL_MUX_H_
