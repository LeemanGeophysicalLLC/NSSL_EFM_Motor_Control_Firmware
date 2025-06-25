#pragma once
#include <Adafruit_FRAM_SPI.h>

/**
 * @brief Persistent configuration structure for the motor controller.
 *
 * Stored in external FRAM.
 */
struct __attribute__((packed)) MotorControllerSettings {
  uint16_t power_cycle_count;     ///< Number of times the system has booted
  uint16_t log_head_index;        ///< Index of the next available log entry in FRAM
  uint16_t current_limit_ma;      ///< Motor current cutoff limit in milliamps
  float pid_kp;                   ///< Proportional gain for motor PID
  float pid_ki;                   ///< Integral gain for motor PID
  float pid_kd;                   ///< Derivative gain for motor PID
  uint16_t setpoint_rpm;          ///< Desired steady-state motor RPM
  uint8_t restart_enabled;        ///< Whether to attempt automatic restart after stall
  uint8_t current_cutoff_enabled; ///< Whether to shut down motor on current over-limit
  // No reserved[] – remove unless you truly need exact layout.
};

/**
 * @brief Global settings instance shared throughout the firmware.
 */
extern MotorControllerSettings settings;

/**
 * @brief Writes the current settings to FRAM.
 *
 * This function enables writing on the FRAM chip, writes the contents of the global
 * `settings` structure to a fixed address, and then disables write mode. It includes
 * a post-write verification read, but does not perform a byte-by-byte compare or error check.
 *
 * @param fram Reference to an initialized Adafruit_FRAM_SPI object.
 */
void writeSettingsToFRAM(Adafruit_FRAM_SPI &fram);

/**
 * @brief Initialize and load settings from FRAM, or reset to defaults if invalid.
 *
 * This function reads the stored configuration from FRAM. If no valid configuration
 * is found (based on firmware-defined criteria), it resets to default values and writes
 * them back to FRAM. Power cycle count is incremented automatically.
 *
 * @param fram Reference to initialized Adafruit_FRAM_SPI object
 */
void initSettings(Adafruit_FRAM_SPI &fram);

/**
 * @brief Save the current global settings structure to FRAM.
 *
 * Intended to be called after user input changes (e.g., via serial commands).
 *
 * @param fram Reference to initialized Adafruit_FRAM_SPI object
 */
void saveSettings(Adafruit_FRAM_SPI &fram);

/**
 * @brief Reset all fields in the settings structure to factory defaults.
 *
 * This does not write to FRAM automatically — call saveSettings() if persistence is needed.
 */
void resetSettings();

/**
 * @brief Print current settings to the serial console.
 *
 * Used for debugging and the `SHOW` command. Outputs formatted, human-readable fields.
 */
void printSettings();

void readSettingsFromFRAM(Adafruit_FRAM_SPI &fram);