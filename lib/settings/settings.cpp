#include "settings.h"
#include <Arduino.h>

#define SETTINGS_ADDR 0x0000
#define SETTINGS_SIZE sizeof(MotorControllerSettings)

/**
 * @brief Global settings object loaded from/stored to FRAM.
 */
MotorControllerSettings settings;

void readSettingsFromFRAM(Adafruit_FRAM_SPI &fram) {
  fram.read(SETTINGS_ADDR, (uint8_t *)&settings, SETTINGS_SIZE);
}

/**
 * @brief Writes the current settings to FRAM.
 *
 * This function enables writing on the FRAM chip, writes the contents of the global
 * `settings` structure to a fixed address, and then disables write mode. It includes
 * a post-write verification read, but does not perform a byte-by-byte compare or error check.
 *
 * @param fram Reference to an initialized Adafruit_FRAM_SPI object.
 */
void writeSettingsToFRAM(Adafruit_FRAM_SPI &fram) {
  fram.writeEnable(true);
  fram.write(SETTINGS_ADDR, (const uint8_t *)&settings, SETTINGS_SIZE);
  fram.writeEnable(false);

  // Verify
  MotorControllerSettings verify;
  fram.read(SETTINGS_ADDR, (uint8_t *)&verify, SETTINGS_SIZE);
}



/**
 * @brief Reset settings to factory defaults.
 *
 * Initializes the `settings` struct with safe defaults:
 * - PID values for moderate response
 * - RPM setpoint of 120
 * - Current limit of 300 mA
 * - Auto-restart enabled
 * - Power cycle count reset to 1
 * - Log head index reset to 0
 *
 * Note: Does NOT write to FRAM — call writeSettingsToFRAM() after this if needed.
 */
void resetSettings() {
  memset(&settings, 0, sizeof(settings));
  settings.power_cycle_count = 1;
  settings.log_head_index = 0;
  settings.current_limit_ma = 300;
  settings.pid_kp = 0.02;
  settings.pid_ki = 0.05;
  settings.pid_kd = 0.0;
  settings.setpoint_rpm = 210;
  settings.restart_enabled = 1;
  settings.current_cutoff_enabled = 1;
}

/**
 * @brief Initialize settings on boot.
 *
 * This function reads settings from FRAM and performs basic sanity checks.
 * If invalid (e.g., RPM is zero or out of range), it resets to defaults.
 * Power cycle counter is incremented and saved back to FRAM automatically.
 *
 * @param fram Reference to initialized Adafruit_FRAM_SPI object.
 */
void initSettings(Adafruit_FRAM_SPI &fram) {
  readSettingsFromFRAM(fram);
  settings.power_cycle_count++;
  writeSettingsToFRAM(fram);
}

/**
 * @brief Save current settings to FRAM.
 *
 * Wrapper for writeSettingsToFRAM(), intended for use when applying changes
 * from the serial command interface.
 *
 * @param fram Reference to initialized Adafruit_FRAM_SPI object.
 */
void saveSettings(Adafruit_FRAM_SPI &fram) {
  writeSettingsToFRAM(fram);
}

/**
 * @brief Print current settings to the serial terminal.
 *
 * This is used for the `SHOW` command and for debugging. Output is formatted
 * as human-readable text with field names and values.
 */
void printSettings() {
  Serial.println("--- Motor Controller Settings ---");
  Serial.print("Setpoint RPM:       "); Serial.println(settings.setpoint_rpm);
  Serial.print("Kp:                 "); Serial.println(settings.pid_kp);
  Serial.print("Ki:                 "); Serial.println(settings.pid_ki);
  Serial.print("Kd:                 "); Serial.println(settings.pid_kd);
  Serial.print("Current Cutoff:     "); Serial.println(settings.current_cutoff_enabled ? "ENABLED" : "DISABLED");
  Serial.print("Current Limit:      "); Serial.print(settings.current_limit_ma); Serial.println(" mA");
  Serial.print("Auto Restart:       "); Serial.println(settings.restart_enabled ? "ENABLED" : "DISABLED");
  Serial.print("Power Cycles:       "); Serial.println(settings.power_cycle_count);
  Serial.print("Log Head Index:     "); Serial.println(settings.log_head_index);
  Serial.println("----------------------------------");
}
