/*
 * NSSL EFM Motor Control Board
 * 
 * This board and firmware are responsible for controlling the motor speed
 * to keep the motor running at a constant RPM even as the batteries run down.
 * The rate of rotation is monitored via a hall effect sensor and magnet on the
 * rotating shaft. The average RPM over several seconds is maintained with new
 * readings being rejected a spurious if they fall outside the average.
 * 
 * The loop runs the PID controller that updates the PWM to the motor every 200ms.
 * Pulses from the hall-effect sensor trigger an ISR that determines the time that
 * revolution took at adds it to the moving average of RPM assuming that it is not
 * more than 10% different from the average. This should help eliminate any signals
 * from discharges or other random events.
 */

#include <Arduino.h>
#include <FIR.h>
#include <PID_v1.h>
#include <IWatchdog.h>
#include "pins.h"
#include "settings.h"
#include "logging.h"
#include <Adafruit_FRAM_SPI.h>
#include "Cmd.h"

#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 0

const int RPM_MIN = 60;
const int RPM_MAX = 300;
const float PID_SETTING_MIN = 0;
const float PID_SETTING_MAX = 100;
const int CURRENT_LIMIT_MIN = 100; // mA
const int CURRENT_LIMIT_MAX = 1000; // mA

// Globals
volatile uint16_t encoder_pulses = 0; // Counts pulses for the encoder 
int16_t temperature = 0;

// PID variables
double feedback = 0;
double pwm_output = 140;
double pid_setpoint = 0;
double pid_kp = 0;
double pid_ki = 0;
double pid_kd = 0;

// Instances
FIR<float, 8> fir;
PID motor_pid(&feedback, &pwm_output, &pid_setpoint,
              pid_kp, pid_ki, pid_kd, DIRECT);
Adafruit_FRAM_SPI fram(PIN_FRAM_CS);

// Function Prototypes
void setRgbLedColor(uint8_t red, uint8_t green, uint8_t blue);
bool CheckArgCount(uint8_t desired_arg_cnt, uint8_t actual_arg_cnt);
void CmdSetRPM(int arg_cnt, char **args);
void CmdSetKP(int arg_cnt, char **args);
void CmdSetKI(int arg_cnt, char **args);
void CmdSetKD(int arg_cnt, char **args);
void CmdSetCurrentLim(int arg_cnt, char **args);
void CmdSetCutoff(int arg_cnt, char **args);
void CmdSetRestart(int arg_cnt, char **args);
void CmdShow(int arg_cnt, char **args);
void CmdResetConfig(int arg_cnt, char **args);
void CmdHelp(int arg_cnt, char **args);
void CmdDumpLog(int arg_cnt, char **args);
int16_t readTemperature();
void Shutdown();
uint16_t CurrentSafetyCheck(uint16_t current_limit_milliamps);
uint16_t ReadBatteryVoltage();
void AttachEncoderInterrupt();
void DetachEncoderInterrupt();
float RPMCheck(uint32_t count_interval_ms);

/**
 * @brief Sets the RGB LED to the specified color.
 *
 * Writes PWM values to the red, green, and blue LED pins using analog output.
 * The brightness of each channel is controlled independently (0–255).
 *
 * @param red   Brightness of the red channel (0–255)
 * @param green Brightness of the green channel (0–255)
 * @param blue  Brightness of the blue channel (0–255)
 */
void setRgbLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  analogWrite(PIN_LED_RED, red);
  analogWrite(PIN_LED_GREEN, green);
  analogWrite(PIN_LED_BLUE, blue);
}

/**
 * @brief Verifies that the expected number of command arguments is provided.
 *
 * Used by command handlers to validate argument count before parsing.
 *
 * @param desired_arg_cnt Expected number of arguments (including command name)
 * @param actual_arg_cnt  Actual number of arguments received
 * @return true if the argument count matches; false otherwise
 */
bool CheckArgCount(uint8_t desired_arg_cnt, uint8_t actual_arg_cnt)
{
  if (desired_arg_cnt == actual_arg_cnt)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Sends a standard OK response over Serial.
 *
 * Used to acknowledge successful command execution.
 */
static inline void cmdOk()
{
  Serial.println(F("OK"));
}

/**
 * @brief Sends a standard error response over Serial.
 *
 * Used to indicate that a command failed or was malformed.
 */
static inline void cmdError()
{
  Serial.println("!");
}

/**
 * @brief Command: SETRPM <rpm>
 * Sets the motor RPM setpoint.
 */
void CmdSetRPM(int arg_cnt, char **args) {
  if (!CheckArgCount(2, arg_cnt)) { cmdError(); return; }
  int rpm = atoi(args[1]);
  if (rpm < RPM_MIN || rpm > RPM_MAX) { cmdError(); return; }
  settings.setpoint_rpm = rpm;
  pid_setpoint = rpm;
  saveSettings(fram);
  readSettingsFromFRAM(fram);
  cmdOk();
}

/**
 * @brief Command: SETKP <value>
 * Sets the PID proportional gain.
 */
void CmdSetKP(int arg_cnt, char **args) {
  if (!CheckArgCount(2, arg_cnt)) { cmdError(); return; }
  float kp = atof(args[1]);
  if (kp < PID_SETTING_MIN || kp > PID_SETTING_MAX) { cmdError(); return; }
  settings.pid_kp = kp;
  pid_kp = kp;
  motor_pid.SetTunings(pid_kp, pid_ki, pid_kd);
  saveSettings(fram);
  cmdOk();
}

/**
 * @brief Command: DUMPLOG
 * Dumps all log entries from FRAM over serial in CSV format.
 */
void CmdDumpLog(int arg_cnt, char **args)
{
  dumpLog();
  cmdOk();
}

/**
 * @brief Command: SETKI <value>
 * Sets the PID integral gain.
 */
void CmdSetKI(int arg_cnt, char **args) {
  if (!CheckArgCount(2, arg_cnt)) { cmdError(); return; }
  float ki = atof(args[1]);
  if (ki < PID_SETTING_MIN|| ki > PID_SETTING_MAX) { cmdError(); return; }
  settings.pid_ki = ki;
  pid_ki = ki;
  motor_pid.SetTunings(pid_kp, pid_ki, pid_kd);
  saveSettings(fram);
  cmdOk();
}

/**
 * @brief Command: SETKD <value>
 * Sets the PID derivative gain.
 */
void CmdSetKD(int arg_cnt, char **args) {
  if (!CheckArgCount(2, arg_cnt)) { cmdError(); return; }
  float kd = atof(args[1]);
  if (kd < PID_SETTING_MIN || kd > PID_SETTING_MAX) { cmdError(); return; }
  settings.pid_kd = kd;
  pid_kd = kd;
  motor_pid.SetTunings(pid_kp, pid_ki, pid_kd);
  saveSettings(fram);
  cmdOk();
}

/**
 * @brief Command: SETCURRENTLIM <milliamps>
 * Sets the motor current limit in milliamps.
 */
void CmdSetCurrentLim(int arg_cnt, char **args) {
  if (!CheckArgCount(2, arg_cnt)) { cmdError(); return; }
  int current = atoi(args[1]);
  if (current < CURRENT_LIMIT_MIN || current > CURRENT_LIMIT_MAX) { cmdError(); return; }
  settings.current_limit_ma = current;
  saveSettings(fram);
  cmdOk();
}

/**
 * @brief Command: SETCUTOFF <0|1>
 * 
 * Enable (1) or disable (0) the current cutoff protection feature.
 * 
 * Usage:
 *   SETCUTOFF 1   --> enables motor shutdown on overcurrent
 *   SETCUTOFF 0   --> disables automatic shutdown
 * 
 * This setting is persistent across reboots (stored in FRAM).
 */
void CmdSetCutoff(int arg_cnt, char **args)
{
  if (!CheckArgCount(2, arg_cnt)) {
    cmdError();
    return;
  }

  int val = atoi(args[1]);
  if (val != 0 && val != 1) {
    cmdError();
    return;
  }

  settings.current_cutoff_enabled = val;
  saveSettings(fram);
  cmdOk();
}

/**
 * @brief Command: SETRESTART <0|1>
 * Enables or disables automatic restart after stall.
 */
void CmdSetRestart(int arg_cnt, char **args) {
  if (!CheckArgCount(2, arg_cnt)) { cmdError(); return; }
  int val = atoi(args[1]);
  if (val != 0 && val != 1) { cmdError(); return; }
  settings.restart_enabled = val;
  saveSettings(fram);
  cmdOk();
}

/**
 * @brief Command: SHOW
 * Prints current configuration values to the serial terminal.
 */
void CmdShow(int arg_cnt, char **args) {
  printSettings();
  cmdOk();
}

/**
 * @brief Command: RESETCONFIG
 * Resets all stored configuration to factory defaults and saves to FRAM.
 */
void CmdResetConfig(int arg_cnt, char **args) {
  resetSettings();
  saveSettings(fram);
  clearLog();  // Wipe the entire log to 0x00
  Serial.println("Settings reset to defaults.");
  cmdOk();
}

/**
 * @brief Lists all available serial commands with brief syntax descriptions.
 *
 * Useful for debugging and operator usage in the field. Outputs to Serial.
 */
void CmdHelp(int arg_cnt, char **args) {
  Serial.println("Available Commands:");
  Serial.println("  SETRPM <value>         - Set RPM target (60–300)");
  Serial.println("  SETKP <float>          - Set Kp gain");
  Serial.println("  SETKI <float>          - Set Ki gain");
  Serial.println("  SETKD <float>          - Set Kd gain");
  Serial.println("  SETCUTOFF <0|1>        - Enable (1) or disable (0) current cutoff protection");
  Serial.println("  SETCURRENTLIM <mA>     - Set current limit (100–1000 mA)");
  Serial.println("  SETRESTART <0|1>       - Enable (1) or disable (0) auto-restart");
  Serial.println("  SHOW                   - Display current settings");
  Serial.println("  RESETCONFIG            - Reset settings to factory defaults");
  Serial.println("  DUMPLOG                - Dump log data as CSV to serial");
  Serial.println("  HELP                   - Show this help message");
  cmdOk();
}

/**
 * @brief Interrupt service routine for encoder pulse detection.
 *
 * Increments the encoder pulse count each time the encoder signal falls.
 */
void EncoderISR()
{
  encoder_pulses += 1;
}

/**
 * @brief Reads the TMP36 temperature sensor and returns temperature in tenths of degrees C.
 *
 * TMP36 outputs 750 mV at 25 °C with a slope of 10 mV/°C.
 * The formula is: T(°C) = (Vout (mV) - 500) / 10
 *
 * @return int16_t Temperature in °C × 10 (e.g., 234 = 23.4°C)
 */
int16_t readTemperature()
{
  
  const float vcc = 3.3;
  const float volts_per_count = vcc / 4096.0;
  int raw = analogRead(PIN_TEMPERATURE_SENSE);    // Replace with the actual analog pin
  float voltage = raw * volts_per_count;

  float temperature_c = (voltage - 0.5) * 100.0;
  return static_cast<int16_t>(temperature_c * 10.0);
}

/**
 * @brief Immediately shuts down motor and optionally restarts after delay.
 *
 * Disables PWM, sets LED to red, waits, then enters infinite loop. 
 * If `restart_enabled` is true, system will be rebooted via watchdog.
 */
void Shutdown()
{
  analogWrite(PIN_MOTOR_PWM, 0);
  setRgbLedColor(255, 0, 0); // Red
  Serial.println("Shutting down.");

  for (uint8_t i=0; i<2; i++)
  {
    IWatchdog.reload();
    delay(20000);
  }
  IWatchdog.reload();
  while(1)
  {
    if (!settings.restart_enabled){IWatchdog.reload();}
  } // Spin until the watchdog catches us if restart is enabled or we just spin forever
}

uint16_t CurrentSafetyCheck(uint16_t current_limit_milliamps)
{
  /**
   * @brief Checks average motor current and shuts down if over limit.
   *
   * @param current_limit_milliamps Shutdown threshold in milliamps.
   * @return uint16_t Current in milliamps.
 */
  uint16_t voltage = 0;
  for (int i=0; i<10; i++)
  {
    voltage += analogRead(PIN_CURRENT_SENSE);
  }
  voltage /= 10;
  // Current = voltage / (Rs * Rl)
  uint16_t current = voltage * 322 / 1000;
  if ((current >= current_limit_milliamps) && settings.current_cutoff_enabled)
  {
    Serial.print("Current limit encountered: ");
    Serial.print(current);
    Serial.println(" mA");
    Shutdown();
  }
  return current;
}

/**
 * @brief Reads the battery voltage from the analog pin and returns millivolts.
 *
 * Uses a 12-bit ADC (0–4095) with a 3.3V reference. The battery voltage is
 * scaled down by a voltage divider and must be multiplied by 3.521 to restore
 * the actual voltage.
 *
 * @return uint16_t Battery voltage in millivolts
 */
uint16_t ReadBatteryVoltage()
{
  const float vref = 3.3;                         // Reference voltage in volts
  const float scale_factor = 3.636;               // Voltage divider multiplier
  const float volts_per_count = vref / 4095.0;    // 12-bit ADC resolution

  int raw = analogRead(PIN_VBAT_SENSE);
  float voltage = raw * volts_per_count * scale_factor;
  return static_cast<uint16_t>(voltage * 1000.0); // Convert to millivolts
}

/**
 * @brief Attaches the interrupt service routine for the motor encoder.
 *
 * Configures an external interrupt on the falling edge of the encoder signal
 * (PIN_MOTOR_ENCODER_A). Each pulse triggers `EncoderISR()` to increment
 * the encoder pulse counter. This is used to measure motor rotation speed.
 */
void AttachEncoderInterrupt()
{
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER_A), EncoderISR, FALLING);
}

/**
 * @brief Detaches the interrupt for the motor encoder.
 *
 * Stops counting encoder pulses by disabling the interrupt on
 * PIN_MOTOR_ENCODER_A. This may be useful during RPM calculation
 * windows or shutdown sequences.
 */
void DetachEncoderInterrupt()
{
  detachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER_A));
}

/**
 * @brief Calculates motor RPM using encoder pulses over a timed interval.
 *
 * @param count_interval_ms Time window in milliseconds to average pulses.
 * @return float RPM value, or -1 if not enough time has passed.
 */
float RPMCheck(uint32_t count_interval_ms)
{
  static uint32_t start_counting = 0;
  static uint32_t stop_counting = 0;

  uint32_t now = millis();

  // Initial setup
  if (start_counting == 0) {
    start_counting = now;
    return -1;
  }

  if ((now - start_counting) >= count_interval_ms)
  {
    DetachEncoderInterrupt();
    stop_counting = now;

    float elapsed_ms = stop_counting - start_counting;
    float elapsed_minutes = elapsed_ms / 60000.0;
    float rpm = (encoder_pulses * 0.0040849) / elapsed_minutes;

    encoder_pulses = 0;
    AttachEncoderInterrupt();
    start_counting = now;

    return rpm;
  }

  return -1;
}

void setup()
{
  /*
   * Setup
   * 
   * This runs once at boot and sets up all of the pin states, classes we'll need, etc.
   */
  pinMode(PIN_MOTOR_ENCODER_A, INPUT);
  pinMode(PIN_MOTOR_ENCODER_B, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  setRgbLedColor(255, 0, 0);   // Red
  delay(500);
  setRgbLedColor(0, 255, 0);   // Green
  delay(500);
  setRgbLedColor(0, 0, 255);   // Blue
  delay(500);
  setRgbLedColor(255, 255, 0); // Yellow


  delay(250);
  IWatchdog.begin(26000000); //max 26208000

  SPI.setMOSI(PIN_FRAM_MOSI);
  SPI.setMISO(PIN_FRAM_MISO);
  SPI.setSCLK(PIN_FRAM_SCK);
  SPI.begin();

  if (fram.begin(2)) // Use 2-byte addressing
  {

  }
  else
  {
    Serial.println("FRAM not found. Check wiring.");
    while (1) {
      setRgbLedColor(255, 0, 0); // Red
      delay(10); // Wait forever
    }
  }

  analogReadResolution(12);

  Serial.begin(9600); // Start a serial port

  initSettings(fram); // Load settings from FRAM or reset to defaults

  Serial.println("NSSL EFM Motor Controller");
  Serial.println("Leeman Geophysical LLC");
  Serial.print("Firmware Version: ");
  Serial.print(FIRMWARE_MAJOR);
  Serial.print(".");
  Serial.println(FIRMWARE_MINOR);

  cmdInit(&Serial);
  cmdAdd("SETRPM", CmdSetRPM);
  cmdAdd("SETKP", CmdSetKP);
  cmdAdd("SETKI", CmdSetKI);
  cmdAdd("SETKD", CmdSetKD);
  cmdAdd("SETCURRENTLIM", CmdSetCurrentLim);
  cmdAdd("SETRESTART", CmdSetRestart);
  cmdAdd("SHOW", CmdShow);
  cmdAdd("RESETCONFIG", CmdResetConfig);
  cmdAdd("DUMPLOG", CmdDumpLog);
  cmdAdd("SETCUTOFF", CmdSetCutoff);
  cmdAdd("HELP", CmdHelp);

  // Logging
  initLogging();

  // PID Setup
  // Copy persistent settings to working doubles
  pid_setpoint = settings.setpoint_rpm;
  pid_kp = settings.pid_kp;
  pid_ki = settings.pid_ki;
  pid_kd = settings.pid_kd;

  // Apply to PID
  motor_pid.SetTunings(pid_kp, pid_ki, pid_kd);
  motor_pid.SetSampleTime(1000);

  // Motor Controller Setup
  //motor.drive(100); // Start the motor
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, HIGH);
  pwm_output = 140;
  analogWrite(PIN_MOTOR_PWM, pwm_output); 
  delay(2000);

  // Make sure the current is okay after spinup
  CurrentSafetyCheck(settings.current_limit_ma);

  IWatchdog.reload();

  // Get the RPM update method running by calling with a short interval
  RPMCheck(1000);
  // Turn on the PID
  motor_pid.SetMode(AUTOMATIC);
  Serial.println("Startup Complete");
  Serial.println("Time_ms, RPM, RPM_Target, PWM, Current_mA, Temp_Cx10, Batt_mV");
}

void loop()
{
  /*
   * Main Loop
   * 
   * The main loop checks if a new rpm reading is available (interrupt has fired) and
   * updates our estimate of the feedback variable. It then calls compute on the PID
   * which will update every 200ms. On update the PWM value is updated and the parameters
   * are printed out on the serial port. We also check the current and if it is over our
   * threshold, we turn off the motor.
   */

  // Pet the dog and check for commands
  IWatchdog.reload();
  cmdPoll();

  // Check the current draw
  uint16_t current_milliamps = CurrentSafetyCheck(settings.current_limit_ma);;

  // Check the battery voltage
  uint16_t battery_voltage = ReadBatteryVoltage();

  // Check if there is an update to the motor RPM
  feedback = RPMCheck(1000);
  feedback = int(feedback);

  if (feedback != -1)
  {
    motor_pid.Compute();  // Compute and update
    temperature = readTemperature(); // Read the temperature sensor
    analogWrite(PIN_MOTOR_PWM, pwm_output);
    Serial.print(millis());
    Serial.print(",");
    Serial.print(feedback);
    Serial.print(",");
    Serial.print(settings.setpoint_rpm);
    Serial.print(",");
    Serial.print(pwm_output);
    Serial.print(",");
    Serial.print(current_milliamps);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    Serial.println(battery_voltage);

    if (abs(settings.setpoint_rpm - feedback) < (settings.setpoint_rpm * 0.01))
    {
      // If we are within 5 RPM of the setpoint, turn on the green LED
      setRgbLedColor(0, 255, 0); // Green
    }
    else
    {
      // If we are not on target, make the LED yellow
      setRgbLedColor(255, 255, 0); // Yellow
    }
  }

  // Write data to the log every 60 seconds
  static uint32_t last_log_time = millis();
  if (millis() - last_log_time >= 60000) {
    Serial.println("Logging data...");
    last_log_time = millis();

    logData(millis() / 1000,          // Timestamp in seconds
            feedback,                 // Current RPM
            current_milliamps,        // Current in mA
            temperature,              // Temperature degC × 10
            battery_voltage,          // mV
            settings.power_cycle_count);
  }
}
