# Drone Project Documentation

## Hardware

### Nucleo-H7A3ZI-Q Settings

#### Clock
- **Sysclk:** 150 MHz

#### Timers

- **TIM1:** 4 channels in PWM generation for motors  
  - **Channels PIN (in order):** PE9, PE11, PE13, PE14  
  - **Prescaler:** 1499  
  - **Counter period:** 1999  

- **TIM2:** Main routine timing  
  - **Global interrupt priority:** 0  

- **TIM3:** PWM input in CH1 (roll)  
  - **Pin input:** PB4  
  - **Prescaler:** 639  
  - **Counter period:** 65535  
  - **Global interrupt priority:** 1  

- **TIM4:** PWM input in CH1 (pitch)  
  - **Pin input:** PB6  
  - **Prescaler:** 639  
  - **Counter period:** 65535  
  - **Global interrupt priority:** 1  

- **TIM5:** PWM input in CH1 (yaw)  
  - **Pin input:** PA0  
  - **Prescaler:** 639  
  - **Counter period:** 65535  
  - **Global interrupt priority:** 1  

- **TIM15:** PWM input in CH1 (throttle)  
  - **Pin input:** PE5  
  - **Prescaler:** 639  
  - **Counter period:** 65535  
  - **Global interrupt priority:** 1  

#### I2C
- **I2C1:**  
  - **Speed frequency:** 400 kHz  
  - **Pins:**  
    - **SCL:** PB8  
    - **SDA:** PB7  

#### Watchdog
- **IWDG1:**  
  - **Counter clock prescaler:** 32  
  - **Down-counter reload:** 999  

#### Radio Controller
- **Flysky FS-T4B:** Mandatory binding with the receiver box.  
- **Hobby King HK TR6A:** Incoming commands modulated on 5–10% duty cycle PWM signals.  

#### ESC
- **Aerostar 30A:** Electronic Stability Control with Battery Eliminator Circuit (BEC)  

#### Motors
- **Turnigy - L2210C-1200kv:** Brushless motors  

#### Battery
- **Turnigy 1.0 1000 mAh Li-Po Battery**

---

## Software

### Main
The `main.c` module serves as the system entry point, coordinating initialization, high-level control flow, and periodic execution of the flight control tasks. It follows the STM32 HAL framework and performs startup routines including system clock configuration and peripheral setup (I2C, UART, GPIOs, timers).

Key subsystems—IMU, motors, orientation estimator, and PID controller—are initialized before entering the main loop. The loop is event-driven, relying on flags (`imu_flag`, `pid_flag`) set by the TIM2 interrupt to regulate timing-sensitive tasks. These include reading IMU data, estimating attitude, computing control outputs, and updating motor PWM, with the specific scheduling already described in the synchronization documentation.

Motor arming is permitted only after the IMU has stabilized and a valid RC signal is continuously received. The RC input is processed in `Handle_RC_Input()`, which decodes PWM duty cycles using timer input capture, converts them to control signals, and refreshes the watchdog. If input is lost or if attitude exceeds a safe limit, motors are disarmed and zeroed.

Motor signals are generated using TIM1 PWM outputs. RC input channels are captured using TIM3, TIM4, TIM5, and TIM15. TIM2 handles the main scheduling (Synchronization).

---

### Control
- Arrays `Kp`, `Ki`, and `Kd` store the proportional, integral, and derivative gains respectively for roll, pitch, and yaw.
- The `pid_update` function uses `HAL_GetTick()` to measure the time (`dt_pid`) elapsed since the last PID update. This is actually fixed since everything is working at 50 Hz. We keep this implementation because of earlier software versions.
- `control_error.p_error[]`, `control_error.d_error[]`, `control_error.i_error[]`:  
  - **Proportional error:** Difference between reference Euler angles and estimated Euler angles from the IMU.  
  - **Derivative error:** Rate of change of the proportional error, computed as the difference between current and previous proportional error divided by the fixed time step.  
  - **Integral error:** Accumulates over time, representing the total deviation from the target.  
- Anti-windup mechanism prevents excessive integrative compensation.
- Previous (`former_error`) values are updated for the next iteration.

---

### IMU
The IMU module handles communication with a 6-DOF inertial sensor over I2C (channel 1), retrieving raw accelerometer and gyroscope data.  
- `imu_init`: Wakes the sensor from sleep by writing to its power management register.  
- `imu_read_word`: Reads a 16-bit value from a specific register.  
- `imu_read_all`: Reads 7 consecutive values representing **Accel X/Y/Z**, **Temp**, and **Gyro X/Y/Z**.  
- Calibration: `imu_calibrate` samples the sensor 100 times to compute average Euler angles (pitch and roll) and gyro values, storing them as offsets to zero the system.  
- Wraparound correction is applied to pitch when needed.

---

### Filter
The Filter module processes IMU data to estimate Euler angles and angular velocity integration.  
- `filter_compute_gravity_angles`: Converts accelerometer readings into roll and pitch using trigonometric projection.  
- `filter_integrate_gyro`: Integrates angular velocity from the gyroscope using trapezoidal approximation, computing `dt_imu` from `HAL_GetTick()`.  
- `filter_fuse_angles`: Blends accelerometer-based and gyro-based estimates with a complementary filter (α = 0.01) for roll and pitch only.  
- Yaw is excluded from fusion since the IMU lacks a magnetometer.

---

### Orientation
The Orientation module continuously estimates drone attitude in Euler angles by combining raw IMU readings with the filtering pipeline.  
- `orientation_init`: Calibrates the IMU and stores the system tick for time tracking.  
- `orientation_update`: Reads IMU data, applies offset corrections, integrates gyro data, and fuses it with accelerometer estimates.  
- Ensures accurate, time-consistent orientation tracking for real-time control.

---

### Motor
The ESCs are powered by the battery and receive a PWM signal from the board, which they convert into a three-phase signal to drive the motors.  
- PWM signals generated using **TIM1** at **50 Hz**, with a duty cycle of 5–10%.  
- Startup:
  - `init_motors`: Sets CCR to 100 (minimum signal, 5% duty cycle).  
  - `set_motor_pwm_zero`: Sets motors to idle.  
- `set_motor_pwm`: Updates CCR values for each channel, clamped between `MOTOR_MAX_PWM` and `MOTOR_MIN_PWM`.

---

### Radio
Each of the four channels (**Pitch**, **Roll**, **Yaw**, **Throttle**) is sampled by an input pin connected to a timer in PWM Input mode.  
- After a valid duty cycle is captured, the interrupt callback normalizes it against predefined scales:
  - Pitch, Roll, Yaw: `[-FULL_RC_SCALE, FULL_RC_SCALE]`  
  - Throttle: `[0, FULL_RC_SCALE]`
- Yaw control has a dead threshold (`YAW_DEAD_THR`) to reduce sensitivity.
- RC calibration (`rc_calibrate`) computes average duty cycles when joysticks are at rest.
- Control starts only if valid commands are received from all channels.
- Watchdog refresh occurs with each valid RC command.  
- If RC is turned off → watchdog triggers reset → system retries calibration but stalls if no input.

---

### Synchronization
To keep all functions synchronized:  
- **TIM2** runs at **200 Hz** and triggers an interrupt (`HAL_TIM_PeriodElapsedCallback`) every 5 ms.

#### Motor PWM Update (50 Hz)
- When `cycle_count == 3` and motors are armed, `set_motor_pwm(motor_pwm)` is called.
- This happens every 20 ms (50 Hz) for consistent motor updates.

#### IMU Data Flag (200 Hz)
- `imu_flag` is set to 1 on every interrupt for high-frequency IMU updates.

#### PID Update Flag (50 Hz, offset)
- When `cycle_count == 1`, `pid_flag` is set to 1 for PID computation.
- Offset from motor update to distribute computational load.

#### Cycle Counter Management
- `cycle_count` increments from 0 to 3, then resets.
- Ensures:
  - IMU integration period = **5 ms**  
  - PID update period = **20 ms**  
  - Motor update period = **20 ms**

---
