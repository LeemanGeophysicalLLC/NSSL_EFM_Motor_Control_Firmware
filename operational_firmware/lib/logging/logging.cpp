// logging.cpp
#include "logging.h"
#include "settings.h"
#include <Arduino.h>
#include <Adafruit_FRAM_SPI.h>

extern Adafruit_FRAM_SPI fram;

#define LOG_ENTRY_SIZE sizeof(LogEntry)
#define LOG_START_ADDR 0x03F0
#define MAX_LOG_ENTRIES (7000 / LOG_ENTRY_SIZE)

struct LogEntry {
  uint32_t timestamp_s;
  int16_t rpm;
  uint16_t current_ma;
  int16_t temp_x10;
  uint16_t battery_mv;
  uint16_t power_cycles;
  uint8_t flags;
  uint8_t reserved[2];
} __attribute__((packed));

/**
 * @brief Write a log entry to FRAM and update the log head pointer.
 *
 * @param timestamp_s Timestamp in seconds since boot
 * @param rpm Current RPM reading
 * @param current Current draw in mA
 * @param temp Temperature x10 in degC
 * @param battery Battery voltage in mV
 * @param power_cycles Number of power cycles
 */
void logData(unsigned long timestamp_s,
             int16_t rpm,
             uint16_t current,
             int16_t temp,
             uint16_t battery,
             uint16_t power_cycles) {
  LogEntry entry;
  entry.timestamp_s = timestamp_s;
  entry.rpm = rpm;
  entry.current_ma = current;
  entry.temp_x10 = temp;
  entry.battery_mv = battery;
  entry.power_cycles = power_cycles;
  entry.flags = 0;

  uint16_t offset = LOG_START_ADDR + settings.log_head_index * LOG_ENTRY_SIZE;
  fram.writeEnable(true);
  fram.write(offset, (uint8_t *)&entry, LOG_ENTRY_SIZE);
  fram.writeEnable(false);

  settings.log_head_index++;
  if (settings.log_head_index >= MAX_LOG_ENTRIES)
    settings.log_head_index = 0;

  saveSettings(fram);
}

/**
 * @brief Dump all logs over serial from oldest to newest
 */
void dumpLog() {
  Serial.println("Timestamp_s,RPM,Current_mA,Temp_degC*10,Battery_mV,PowerCycles");

  uint16_t index = settings.log_head_index;

  for (uint16_t i = 0; i < MAX_LOG_ENTRIES; ++i) {
    uint16_t offset = LOG_START_ADDR + index * LOG_ENTRY_SIZE;
    LogEntry entry;
    fram.read(offset, (uint8_t *)&entry, LOG_ENTRY_SIZE);

    Serial.print(entry.timestamp_s);
    Serial.print(",");
    Serial.print(entry.rpm);
    Serial.print(",");
    Serial.print(entry.current_ma);
    Serial.print(",");
    Serial.print(entry.temp_x10 / 10.0);
    Serial.print(",");
    Serial.print(entry.battery_mv);
    Serial.print(",");
    Serial.println(entry.power_cycles);

    index++;
    if (index >= MAX_LOG_ENTRIES) index = 0;
  }
}





/**
 * @brief Clears all stored log entries in FRAM by overwriting with 0xFF.
 */
void clearLog() {
  Serial.print("Log entry size: ");
  Serial.println(LOG_ENTRY_SIZE);
  Serial.print("Log start address: ");
  Serial.println(LOG_START_ADDR, HEX);
  Serial.print("Max log entries: ");
  Serial.println(MAX_LOG_ENTRIES);
  
  for (uint16_t i = 0; i < LOG_ENTRY_SIZE * MAX_LOG_ENTRIES; ++i) {
    fram.writeEnable(true);
    fram.write8(LOG_START_ADDR + i, 0x00);
    fram.writeEnable(false);
  }
  settings.log_head_index = 0;
  saveSettings(fram);

  Serial.println("Log cleared.");
}


/**
 * @brief Initialize logging system.
 *
 * Currently a no-op, but retained for future expansion (e.g., log validation,
 * rollover marker check, or boot-time flags).
 */
void initLogging() {
  // No action required for now.
  // Placeholder for future integrity checks or boot-time log handling.
}
