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

// Globals
volatile uint16_t encoder_pulses = 0; // Counts pulses for the encoder 
char motor_cutoff_enable = 1;
char tone_system_enable = 1;
double setpoint = 120; // Target RPM for the motor
double feedback = 0; // Feedback for the motor PID
double pwm_output = 80; // Output from the PID
double pid_kp=0.5;
double pid_ki=0.07;
double pid_kd=0.0;
uint16_t motor_current_limit = 300; // mA limit before motor cutoff

// Instances
FIR<float, 4> fir;  // Since we are shooting for about 120 RPM this is a 10 second average.
PID motor_pid(&feedback, &pwm_output, &setpoint, pid_kp, pid_ki, pid_kd, REVERSE);

void EncoderISR()
{
  /*
   * Encoder ISR
   * 
   * This ISR encrements encoder pulse counts.
   */
  encoder_pulses += 1;
}

void PlayTones()
{
  /*
   * Play Tones
   * 
   * Play tones to aid in location of the instrument.
   */
    tone(PIN_BUZZER, 1000);
    delay(1000);
    tone(PIN_BUZZER, 2000);
    delay(1000);
    tone(PIN_BUZZER, 3000);
    delay(1000);
    tone(PIN_BUZZER, 4000);
    delay(1000);
    tone(PIN_BUZZER, 5000);
    delay(1000);
    noTone(PIN_BUZZER);
}

void Shutdown()
{
  /*
   * Shutdown
   * 
   * Turns off the motor, plays tones, then tries a restart of everything after ~1 minute.
   */
  analogWrite(PIN_MOTOR_PWM, 255);
  Serial.println("Shutting down.");
  if (tone_system_enable)
  {
    PlayTones();
  }
  for (uint8_t i=0; i<2; i++)
  {
    IWatchdog.reload();
    delay(20000);
  }
  IWatchdog.reload();
  while(1){} // Spin until the watchdog catches us!
}

uint16_t CurrentSafetyCheck(uint16_t current_limit_milliamps)
{
  /*
   * Current Safety Check
   * 
   * Checks the current draw of the motor and if it is over the given threshold we
   * go to the shutdown state if enabled. Otherwise return to where we were.
   */
  uint16_t voltage = 0;
  for (int i=0; i<10; i++)
  {
    voltage += analogRead(PIN_CURRENT_SENSE);
  }
  voltage /= 10;
  // Current = voltage / (Rs * Rl)
  uint16_t current = voltage * 322 / 1000;
  if ((current >= current_limit_milliamps) && motor_cutoff_enable)
  {
    Serial.print("Current limit encountered: ");
    Serial.print(current);
    Serial.println(" mA");
    Shutdown();
  }
  return current;
}

uint16_t ReadBatteryVoltage()
{
  /*
   * Read Battery Voltage
   * 
   * Check the battery voltage and return it as a number in millivolts.
   * Second multiplier is to account for the hardware voltage divider.
   */
  return analogRead(PIN_VBAT_SENSE) * 3.22656 * 3.7037;
}

void AttachEncoderInterrupt()
{
  /*
   * Start the encoder pulse counting ISR.
   */
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER), EncoderISR, FALLING);
}


void DetachEncoderInterrupt()
{
  /*
   * Stop the encoder pulse counting ISR.
   */
  detachInterrupt(digitalPinToInterrupt(PIN_MOTOR_ENCODER));
}


float_t CalculateRPM(uint16_t count, uint32_t start_time, uint32_t end_time)
{
  /*
   * Calculate the RPM of the motor given 6 pulses/rev and the start/end times.
   */
  return (count / 6.0) / ((end_time - start_time) / 60000);
}

float RPMCheck(uint32_t count_interval_ms)
{
  /* Checks on the RPM of the motor - if we are counting, see if we should keep counting.
   * If we are still counting, return -1. If time has elapsed, we stop counting,
   * calculate RPM, restart counting, return the RPM.
   */
  static uint32_t start_counting = millis();
  static uint32_t stop_counting = millis();

  // If it's time to do the RPM update
  if ((millis() - start_counting) >= count_interval_ms)
  {
    // Stop the interrupt, do the math
    DetachEncoderInterrupt();
    stop_counting = millis();
    float rpm = (encoder_pulses / 6.0) * ((stop_counting - start_counting) / 60000.0);
    encoder_pulses = 0;

    // Restart the interrupt
    AttachEncoderInterrupt();
    start_counting = millis();

    return rpm;
  }

  // Not time to do the update yet
  return -1;
}

void VerifyRotation()
{
  // Verify that the motor is rotating at least a little (1 pulse/100ms). Blocking.
  if (motor_cutoff_enable)
  {
   uint16_t encoder_initial = encoder_pulses;
   delay(100);
   uint16_t encoder_final = encoder_pulses;
   if ((encoder_final - encoder_initial) <= 0)
   {
     Shutdown();
   }
  }
}

void setup()
{
  /*
   * Setup
   * 
   * This runs once at boot and sets up all of the pin states, classes we'll need, etc.
   */

  IWatchdog.begin(26000000); //max 26208000

  delay(2000);  // Wait incase a user is starting a terminal
  Serial.begin(115200); // Start a serial port
  Serial.println("NSSL EFM Motor Controller");
  Serial.println("Leeman Geophysical LLC");
  Serial.println("Setting up controller...");

  // Pin Setup
  pinMode(PIN_MOTOR_ENCODER, INPUT);
  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);
  pinMode(PIN_TONE_SYSTEM_ENABLE, INPUT);
  pinMode(PIN_MOTOR_CUTOFF_ENABLE, INPUT);

  
  // PID Setup
  motor_pid.SetSampleTime(9000);

  // Read the dip switch settings - this only happens once on boot!
  motor_cutoff_enable = !digitalRead(PIN_MOTOR_CUTOFF_ENABLE);
  tone_system_enable = !digitalRead(PIN_TONE_SYSTEM_ENABLE);
  
  if (motor_cutoff_enable){Serial.println("Motor Cutoff Enabled.");}
  else {Serial.println("Motor Cutoff Disabled.");}

  if (tone_system_enable){Serial.println("Tone System Enabled.");}
  else {Serial.println("Tone System Disabled.");}

  // Motor Controller Setup
  digitalWrite(PIN_MOTOR_DIRECTION, LOW);
  analogWrite(PIN_MOTOR_PWM, 10); // Start the motor
  delay(2000);

  // Make sure the current is okay after spinup
  CurrentSafetyCheck(motor_current_limit);

  IWatchdog.reload();

  // Get the RPM update method running by calling with a short interval
  RPMCheck(1000);
  delay(1100);
  RPMCheck(1000);
  delay(1100);
  // Turn on the PID
  motor_pid.SetMode(AUTOMATIC);
  Serial.println("Startup Complete");
  Serial.println("Milliseconds, Feedback, Setpoint, PWM_Output, Current_mA, Battery_mV");

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

  IWatchdog.reload();

  // Check the current draw
  uint16_t current_milliamps = CurrentSafetyCheck(motor_current_limit);

  // Check the battery voltage
  uint16_t battery_voltage = ReadBatteryVoltage();

  // Check that in 100ms we at least get 1 encoder pulse - otherwise stop!
  VerifyRotation();

  // Check if there is an update to the motor RPM
  feedback = RPMCheck(10000);
  feedback = int(feedback);

  if (feedback != -1)
  {
    motor_pid.Compute();  // Compute and update
    analogWrite(PIN_MOTOR_PWM, pwm_output);
    Serial.print(millis());
    Serial.print(",");
    Serial.print(feedback);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(pwm_output);
    Serial.print(",");
    Serial.print(current_milliamps);
    Serial.print(",");
    Serial.println(battery_voltage);
  }
}