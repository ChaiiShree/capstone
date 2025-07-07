#include <Wire.h>
#include <ESP32Servo.h> // ESP32-specific servo library

// Gyroscope rate variables
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

// ESC frequency setting
int ESCfreq = 500;

// PID coefficients for angle mode (outer loop)
float PAngleRoll = 2; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007; float DAnglePitch = DAngleRoll;

// PID coefficients for rate mode (inner loop)
float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
float t = 0.004;      // Loop time cycle in seconds (250Hz)

// Motor objects - Modified for hexacopter (6 motors)
Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
Servo mot5; // Added for hexacopter
Servo mot6; // Added for hexacopter

// Define motor pin assignments for A2212 BLDC motors
const int mot1_pin = 5;
const int mot2_pin = 13;
const int mot3_pin = 18;
const int mot4_pin = 19;
const int mot5_pin = 23;
const int mot6_pin = 25;

// RC receiver variables
volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6]; // Array for 6 channels

// RC receiver pin definitions - Using alternative pins to avoid conflicts
const int channel_1_pin = 34;  // Roll
const int channel_2_pin = 35;  // Pitch 
const int channel_3_pin = 4;   // Throttle - Changed from 32
const int channel_4_pin = 2;   // Yaw - Changed from 33
const int channel_5_pin = 15;  // Mode - Changed from 25 (ESC conflict)
const int channel_6_pin = 14;  // Aux - Changed from 26 (ESC conflict)

// PID calculation variables
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float KalmanGainPitch, KalmanGainRoll;

// Throttle settings
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

// Flight control variables
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Accelerometer and angle variables for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Complementary filter variables
float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

// Motor output variables - Modified for hexacopter
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4, MotorInput5, MotorInput6;

// Kalman filter implementation
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // Variance of IMU (4 deg/s)
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // Standard deviation of error is 3 deg
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

// RC receiver interrupt handler
void channelInterruptHandler() {
  current_time = micros(); 
  if (digitalRead(channel_1_pin)) { 
    if (last_channel_1 == 0) { 
      last_channel_1 = 1; 
      timer_1 = current_time; 
    } 
  } else if (last_channel_1 == 1) { 
    last_channel_1 = 0; 
    ReceiverValue[0] = current_time - timer_1; 
  } 
  
  if (digitalRead(channel_2_pin)) { 
    if (last_channel_2 == 0) { 
      last_channel_2 = 1; 
      timer_2 = current_time; 
    } 
  } else if (last_channel_2 == 1) { 
    last_channel_2 = 0; 
    ReceiverValue[1] = current_time - timer_2; 
  } 
  
  if (digitalRead(channel_3_pin)) { 
    if (last_channel_3 == 0) { 
      last_channel_3 = 1; 
      timer_3 = current_time; 
    } 
  } else if (last_channel_3 == 1) { 
    last_channel_3 = 0; 
    ReceiverValue[2] = current_time - timer_3; 
  } 
  
  if (digitalRead(channel_4_pin)) { 
    if (last_channel_4 == 0) { 
      last_channel_4 = 1; 
      timer_4 = current_time; 
    } 
  } else if (last_channel_4 == 1) { 
    last_channel_4 = 0; 
    ReceiverValue[3] = current_time - timer_4; 
  } 
  
  if (digitalRead(channel_5_pin)) { 
    if (last_channel_5 == 0) { 
      last_channel_5 = 1; 
      timer_5 = current_time; 
    } 
  } else if (last_channel_5 == 1) { 
    last_channel_5 = 0; 
    ReceiverValue[4] = current_time - timer_5; 
  } 
  
  if (digitalRead(channel_6_pin)) { 
    if (last_channel_6 == 0) { 
      last_channel_6 = 1; 
      timer_6 = current_time; 
    } 
  } else if (last_channel_6 == 1) { 
    last_channel_6 = 0; 
    ReceiverValue[5] = current_time - timer_6; 
  }
}

void gyro_signals(void)
{
  // Configure MPU6050 sensor settings
  Wire.beginTransmission(0x68);  // Start communication with MPU6050 (0x68 is its I2C address)
  Wire.write(0x1A);              // Access the CONFIG register
  Wire.write(0x03);              // Set Digital Low Pass Filter to 3 (42Hz for gyro)
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);              // Access the ACCEL_CONFIG register
  Wire.write(0x10);              // Set accelerometer to ±8g range (0x10 = 0b00010000)
  Wire.endTransmission();
  
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);              // Start reading from ACCEL_XOUT_H register
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);     // Request 6 bytes (3 axes, 2 bytes each)
  
  // Combine high and low bytes for each accelerometer axis
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();  // X-axis
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();  // Y-axis
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();  // Z-axis
  
  // Configure gyroscope settings
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);              // Access the GYRO_CONFIG register
  Wire.write(0x8);               // Set gyroscope to ±500 degrees/s range (0x08 = 0b00001000)
  Wire.endTransmission();
  
  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);              // Start reading from GYRO_XOUT_H register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);     // Request 6 bytes (3 axes, 2 bytes each)
  
  // Combine high and low bytes for each gyroscope axis
  int16_t GyroX = Wire.read() << 8 | Wire.read();  // Roll rate
  int16_t GyroY = Wire.read() << 8 | Wire.read();  // Pitch rate
  int16_t GyroZ = Wire.read() << 8 | Wire.read();  // Yaw rate
  
  // Convert raw values to usable units
  // For ±500 degrees/s range, sensitivity is 65.5 LSB/(degrees/s)
  RateRoll = (float)GyroX / 65.5;    // Convert to degrees/second
  RatePitch = (float)GyroY / 65.5;   // Convert to degrees/second
  RateYaw = (float)GyroZ / 65.5;     // Convert to degrees/second
  
  // For ±8g range, sensitivity is 4096 LSB/g
  AccX = (float)AccXLSB / 4096;      // Convert to g units
  AccY = (float)AccYLSB / 4096;      // Convert to g units
  AccZ = (float)AccZLSB / 4096;      // Convert to g units
  
  // Calculate roll and pitch angles from accelerometer data using trigonometry
  // 57.29 is the conversion factor from radians to degrees (180/π)
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  // For hexacopter: No change needed in sensor reading, but we'll add a comment
  // Hexacopter uses the same orientation sensing as quadcopter
}

// Enhanced PID function with anti-windup and additional comments for hexacopter
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  // Proportional term - directly proportional to error
  float Pterm = P * Error;
  
  // Integral term - accumulates error over time (trapezoidal integration)
  // Uses previous I term and adds new area under error curve
  float Iterm = PrevIterm + (I * (Error + PrevError) * (t/2));
  
  // Anti-windup - limit the integral term to prevent excessive buildup
  // This is crucial for stable flight, especially for hexacopters with more motors
  if (Iterm > 400) {
    Iterm = 400;
  } else if (Iterm < -400) {
    Iterm = -400;
  }
  
  // Derivative term - based on rate of change of error
  // Helps dampen oscillations and improve stability
  float Dterm = D * ((Error - PrevError) / t);
  
  // Combine all terms to get final PID output
  float PIDOutput = Pterm + Iterm + Dterm;
  
  // Limit the total output to prevent excessive commands
  // For hexacopter: Same limits work well, as individual motor mixing will be handled separately
  if (PIDOutput > 400) {
    PIDOutput = 400;
  } else if (PIDOutput < -400) {
    PIDOutput = -400;
  }
  
  // Store results for return and future use
  PIDReturn[0] = PIDOutput;  // The actual PID output
  PIDReturn[1] = Error;      // Current error (for next iteration)
  PIDReturn[2] = Iterm;      // Current I term (for next iteration)
}

void setup(void) {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // LED startup sequence - visual indicator that the system is initializing
  int led_time = 100;
  pinMode(15, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
  }
  digitalWrite(15, LOW);

  // Configure RC receiver input pins with pull-up resistors
  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);
  pinMode(channel_6_pin, INPUT_PULLUP);

  // Attach interrupt handlers to receiver pins to detect signal changes
  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);
  delay(100);
  
  // Initialize I2C communication with the IMU at 400kHz
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  // Wake up the MPU6050 IMU (register 0x6B - Power Management 1)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);  // Set to 0 to wake up the IMU
  Wire.endTransmission();

  // Allocate ESP32 hardware timers for the servo/ESC control
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(1000);
  
  // Initialize all six motors for hexacopter (adding motors 5 and 6)
  // Motor 1
  mot1.attach(mot1_pin, 1000, 2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  
  // Motor 2
  mot2.attach(mot2_pin, 1000, 2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  
  // Motor 3
  mot3.attach(mot3_pin, 1000, 2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  
  // Motor 4
  mot4.attach(mot4_pin, 1000, 2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);
  
  // Motor 5 (new for hexacopter)
  mot5.attach(mot5_pin, 1000, 2000);
  delay(1000);
  mot5.setPeriodHertz(ESCfreq);
  delay(100);
  
  // Motor 6 (new for hexacopter)
  mot6.attach(mot6_pin, 1000, 2000);
  delay(1000);
  mot6.setPeriodHertz(ESCfreq);
  delay(100);

  // Set all motors to minimum throttle (arm the ESCs)
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  mot5.writeMicroseconds(1000);  // New for hexacopter
  mot6.writeMicroseconds(1000);  // New for hexacopter
  
  delay(500);
  
  // Final LED confirmation sequence
  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);

  // Set sensor calibration values
  // These values compensate for sensor bias/offset
  RateCalibrationRoll = 0.27;
  RateCalibrationPitch = -0.85;
  RateCalibrationYaw = -2.09;
  AccXCalibration = 0.03;
  AccYCalibration = 0.01;
  AccZCalibration = -0.07;

  // Initialize loop timer for consistent cycle time
  LoopTimer = micros();
}

void loop(void) {
  // Read sensor data from MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;

  // Apply calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // Calculate angles from accelerometer data
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;

  // Apply complementary filter for angle estimation
  complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
  complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
  
  // Limit angles to ±20 degrees for safety
  complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

  // Process receiver inputs to get desired values
  DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
  DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
  InputThrottle=ReceiverValue[2];
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);

  // Angle PID controllers (outer loop) - Roll
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Angle PID controllers (outer loop) - Pitch
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Compute errors for rate PID
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Rate PID controllers (inner loop) - Roll
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Rate PID controllers (inner loop) - Pitch
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Rate PID controllers (inner loop) - Yaw
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // Limit throttle
  if (InputThrottle > 1800) {
    InputThrottle = 1800;
  }

  // Hexacopter motor mixing (assuming flat hex configuration)
  // Motors are numbered clockwise from front:
  // Motor1: Front
  // Motor2: Front Right
  // Motor3: Rear Right
  // Motor4: Rear
  // Motor5: Rear Left
  // Motor6: Front Left
  
  MotorInput1 = InputThrottle - InputPitch * 0.866 - InputYaw;                  // Front (CW)
  MotorInput2 = InputThrottle - InputRoll * 0.866 - InputPitch * 0.5 + InputYaw; // Front Right (CCW)
  MotorInput3 = InputThrottle - InputRoll * 0.866 + InputPitch * 0.5 - InputYaw; // Rear Right (CW)
  MotorInput4 = InputThrottle + InputPitch * 0.866 + InputYaw;                  // Rear (CCW)
  MotorInput5 = InputThrottle + InputRoll * 0.866 + InputPitch * 0.5 - InputYaw; // Rear Left (CW)
  MotorInput6 = InputThrottle + InputRoll * 0.866 - InputPitch * 0.5 + InputYaw; // Front Left (CCW)

  // Apply upper limits to all motors
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;
  if (MotorInput5 > 2000) MotorInput5 = 1999;
  if (MotorInput6 > 2000) MotorInput6 = 1999;

  // Apply minimum throttle when armed
  if (InputThrottle >= 1030) {
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;
    if (MotorInput5 < ThrottleIdle) MotorInput5 = ThrottleIdle;
    if (MotorInput6 < ThrottleIdle) MotorInput6 = ThrottleIdle;
  } else {
    // Disarm motors and reset PID terms
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    MotorInput5 = ThrottleCutOff;
    MotorInput6 = ThrottleCutOff;
    
    // Reset PID terms to prevent integral windup
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;    
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
  }

  // Write values to motors
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);
  mot5.writeMicroseconds(MotorInput5);
  mot6.writeMicroseconds(MotorInput6);

  // Maintain consistent loop timing
  while (micros() - LoopTimer < (t*1000000)) {
    // Wait until the loop time is reached
  }
  LoopTimer = micros();
}
