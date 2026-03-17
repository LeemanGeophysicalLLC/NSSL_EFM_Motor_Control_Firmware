#pragma once
#include <stdint.h>

/**
 * @file logging.h
 * @brief Motor data logging interface for FRAM-based storage.
 */

/**
 * @brief Initializes the logging system and checks FRAM readiness.
 * 
 * Should be called once at boot after FRAM and settings are initialized.
 */
void initLogging();

/**
 * @brief Log a new data point to FRAM.
 * 
 * Should be called at fixed intervals (e.g., every 60 seconds).
 * 
 * @param timestamp_s     Uptime in seconds.
 * @param rpm             Current motor RPM.
 * @param current         Motor current in milliamps.
 * @param temp            Controller temperature in deci-degrees C.
 * @param battery         Battery voltage in millivolts.
 * @param power_cycles    Power cycle counter from settings.
 */
void logData(unsigned long timestamp_s,
             int16_t rpm,
             uint16_t current,
             int16_t temp,
             uint16_t battery,
             uint16_t power_cycles);


/**
 * @brief Dump all logged data via serial in chronological order.
 * 
 * Starts from the oldest entry and continues to the newest, wrapping around
 * if necessary. Output is printed to the current active Serial port.
 */
void dumpLog();

/**
 * @brief Clears the log stored in FRAM and resets the write index.
 * 
 * This function overwrites all log entries with 0x00 (uninitialized state)
 * and resets the log_head_index to zero. Useful for factory reset or testing.
 */
void clearLog();
