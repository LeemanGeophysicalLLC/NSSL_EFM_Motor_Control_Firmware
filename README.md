# NSSL EFM Motor Control Firmware

One of the paddles on the instrument houses a motor driver, which reads motor speed  
from an encoder and controls power to the motor to maintain a constant rotation rate  
of the instrument. We also monitor the current supplied to the motor and the  
temperature of the control board, logging all of this data to a rolling log file.  
This log is used for debugging and diagnosing any flight-related failures.

See the [main instrument repo](https://github.com/LeemanGeophysicalLLC/NSSL_EFM_Instrument)
for documentation and the rest of the instrument.
