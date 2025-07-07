#include <Wire.h>

// Sensor data variables
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccZCalibration, AccYCalibration, AccXCalibration;
int RateCalibrationNumber;

// Variables to store the calibrated values
float calAccX, calAccY, calAccZ;

// A2212 motor-specific vibration thresholds
const float VIBRATION_THRESHOLD = 0.05; // g
bool vibrationDetected = false;

// CT6B transmitter settings
const int CT6B_CENTER = 1500;
const int CT6B_DEADBAND = 15; // Wider deadband for CT6B's lower resolution

// Hexacopter motor configuration for A2212 BLDC motors
const int motorPins[6] = {5, 13, 18, 19, 23, 25}; // Motor pin assignments
const float SIN_60 = 0.866;  // sin(60°)
const float SIN_30 = 0.5;    // sin(30°)

// Hexacopter-specific motor mixing factors
float hexMotorMixFactors[6][3] = {
  {0.0, -0.866, -1.0},    // Motor 1: Front (pitch, roll, yaw)
  {-0.866, -0.5, 1.0},    // Motor 2: Front Right
  {-0.866, 0.5, -1.0},    // Motor 3: Rear Right
  {0.0, 0.866, 1.0},      // Motor 4: Rear
  {0.866, 0.5, -1.0},     // Motor 5: Rear Left
  {0.866, -0.5, 1.0}      // Motor 6: Front Left
};

// Variables for vibration detection
float accXHistory[10], accYHistory[10], accZHistory[10];
int historyIndex = 0;
float vibrationLevel = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // LED for status indication
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // Initialize the MPU6050 sensor with settings optimized for A2212 motors
  Wire.beginTransmission(0x68); // MPU6050 address
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);
  
  // Configure gyro for higher vibration tolerance (A2212 motors can cause vibration)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x05); // Set DLPF to 5 (10Hz bandwidth) for better filtering
  Wire.endTransmission();
  
  // Configure accelerometer for higher g-range (A2212 motors can cause higher acceleration)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x10); // Set to ±8g range
  Wire.endTransmission();
  
  delay(100);
  
  // Welcome message with specific hardware information
  Serial.println("A2212 BLDC Motors - Hexacopter IMU Calibration");
  Serial.println("Compatible with FlySky CT6B Transmitter");
  Serial.println("---------------------------------------------");
  Serial.println("Place the hexacopter flat on a surface and press 'Space bar' and then 'Enter' to begin calibration.");
  Serial.println("IMPORTANT: Ensure the hexacopter is completely still during calibration.");
  
  // Wait for user input
  while (!Serial.available()) {
    // Blink LED while waiting
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  Serial.read();  // Clear the input buffer
  
  // Perform extended calibration for A2212 motors (more samples)
  Serial.println("Starting extended calibration for A2212 motors...");
  digitalWrite(13, HIGH); // LED on during calibration
  calibrateGyroExtended();
  digitalWrite(13, LOW);
  
  // Print calibration results
  Serial.println("\nCalibration Results:");
  Serial.print("GyroX: ");
  Serial.print(RateRoll);
  Serial.print(", GyroY: ");
  Serial.print(RatePitch);
  Serial.print(", GyroZ: ");
  Serial.print(RateYaw);
  Serial.print(", AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.println(AccZ);
  Serial.println("Calibration complete!");

  // Print calibration values in a format that can be copied to code
  Serial.println("\nCopy these values to your flight controller code:");
  Serial.print("RateCalibrationRoll=");
  Serial.print(RateCalibrationRoll, 6);
  Serial.println(";");
  Serial.print("RateCalibrationPitch=");
  Serial.print(RateCalibrationPitch, 6);
  Serial.println(";");
  Serial.print("RateCalibrationYaw=");
  Serial.print(RateCalibrationYaw, 6);
  Serial.println(";");
  Serial.print("AccXCalibration=");
  Serial.print(AccXCalibration, 6);
  Serial.println(";");
  Serial.print("AccYCalibration=");
  Serial.print(AccYCalibration, 6);
  Serial.println(";");
  Serial.print("AccZCalibration=");
  Serial.print(AccZCalibration, 6);
  Serial.println(";");
  
  // Print hexacopter motor configuration
  Serial.println("\nHexacopter Motor Configuration (A2212 BLDC Motors):");
  for (int i = 0; i < 6; i++) {
    Serial.print("Motor ");
    Serial.print(i+1);
    Serial.print(" (Pin ");
    Serial.print(motorPins[i]);
    Serial.print("): Pitch=");
    Serial.print(hexMotorMixFactors[i][0], 3);
    Serial.print(", Roll=");
    Serial.print(hexMotorMixFactors[i][1], 3);
    Serial.print(", Yaw=");
    Serial.println(hexMotorMixFactors[i][2], 3);
  }
  
  // Print CT6B transmitter settings
  Serial.println("\nFlySky CT6B Transmitter Settings:");
  Serial.println("Center Position: " + String(CT6B_CENTER) + "μs");
  Serial.println("Deadband: ±" + String(CT6B_DEADBAND) + "μs");
  Serial.println("Note: CT6B has lower resolution (1024) than higher-end transmitters");
  
  Serial.println("\nPress 'Space bar' and then 'Enter' to begin continuous monitoring");
  while (!Serial.available()) {
    // Blink LED while waiting
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
  Serial.read();
  
  Serial.println("Starting continuous monitoring...");
  Serial.println("---------------------------------------------");
}

void loop() {
  gyro_signals();

  // Apply calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // Calculate angles from accelerometer data
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29; // Convert to degrees
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29; // Convert to degrees
  
  // Check for vibration (important for A2212 motors)
  checkVibration();
  
  // Print formatted sensor data
  Serial.print("Acc [g]: X=");
  Serial.print(AccX, 3);
  Serial.print(" Y=");
  Serial.print(AccY, 3);
  Serial.print(" Z=");
  Serial.print(AccZ, 3);
  
  Serial.print(" | Gyro [°/s]: X=");
  Serial.print(RateRoll, 1);
  Serial.print(" Y=");
  Serial.print(RatePitch, 1);
  Serial.print(" Z=");
  Serial.print(RateYaw, 1);
  
  Serial.print(" | Angle [°]: Roll=");
  Serial.print(AngleRoll, 1);
  Serial.print(" Pitch=");
  Serial.print(AnglePitch, 1);
  
  // Show vibration level (important for A2212 motors)
  Serial.print(" | Vib=");
  Serial.print(vibrationLevel, 4);
  if (vibrationDetected) {
    Serial.println(" WARNING: High vibration detected!");
    digitalWrite(13, HIGH); // LED on to indicate vibration
  } else {
    Serial.println();
    digitalWrite(13, LOW);
  }

  delay(50); // Faster update rate for better monitoring
}

void gyro_signals(void) {
  // Configure digital low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05); // Set to 10Hz bandwidth for A2212 motors
  Wire.endTransmission();
  
  // Configure accelerometer range
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10); // Set to ±8g range for A2212 motors
  Wire.endTransmission();
  
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();

  // Configure gyroscope range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08); // Set to ±500°/s range
  Wire.endTransmission();
  
  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert raw values to usable units
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
}

// Extended calibration function specifically for A2212 motors
void calibrateGyroExtended() {
  const int samples = 3000; // More samples for better accuracy with A2212 motors
  
  Serial.println("Keep hexacopter perfectly still...");
  delay(1000); // Allow time to stabilize
  
  // Reset calibration values
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;
  AccXCalibration = 0;
  AccYCalibration = 0;
  AccZCalibration = 0;
  
  // Take multiple samples
  for (RateCalibrationNumber = 0; RateCalibrationNumber < samples; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccXCalibration += AccX;
    AccYCalibration += AccY;
    AccZCalibration += AccZ;
    
    // Show progress
    if (RateCalibrationNumber % 300 == 0) {
      Serial.print(".");
    }
    
    delay(2); // Slightly longer delay for more stable readings
  }
  
  // Calculate averages
  RateCalibrationRoll /= samples;
  RateCalibrationPitch /= samples;
  RateCalibrationYaw /= samples;
  AccXCalibration /= samples;
  AccYCalibration /= samples;
  AccZCalibration /= samples;
  
  // Adjust Z acceleration to read 1g when level
  AccZCalibration = AccZCalibration - 1;
  
  Serial.println("\nCalibration complete!");
}

void calibrateAcc() {
  //AccY calc
  Serial.println("Place the hexacopter on its Right side with nose forward and press 'Space bar' and then 'Enter'.");
  while (!Serial.available()) {}
  Serial.read();
  gyro_signals(); 
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.println(AccZ);
  AccYCalibration = AccY - 1;  
  
  //Acc X calc
  Serial.println("Place the hexacopter nose up and press 'Space bar' and then 'Enter'.");
  while (!Serial.available()) {}
  Serial.read();
  gyro_signals();
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.println(AccZ);
  AccXCalibration = AccX - 1;  
}

void calibrateGyro() {
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 1000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccZCalibration +=  AccZ;
    delay(1);
  }
  RateCalibrationRoll /= 1000;
  RateCalibrationPitch /= 1000;
  RateCalibrationYaw /= 1000;
  AccZCalibration /= 1000;
  AccZCalibration = AccZCalibration - 1;
}

void calibrateGyroSimple() {
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccXCalibration += AccX;
    AccYCalibration += AccY;
    AccZCalibration += AccZ;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  AccXCalibration /= 2000;
  AccYCalibration /= 2000;
  AccZCalibration /= 2000;
  AccZCalibration = AccZCalibration - 1;
}

// New function to detect vibrations from A2212 motors
void checkVibration() {
  // Store acceleration history
  accXHistory[historyIndex] = AccX;
  accYHistory[historyIndex] = AccY;
  accZHistory[historyIndex] = AccZ;
  historyIndex = (historyIndex + 1) % 10;
  
  // Calculate variance (measure of vibration)
  float sumVarX = 0, sumVarY = 0, sumVarZ = 0;
  float avgX = 0, avgY = 0, avgZ = 0;
  
  // Calculate averages
  for (int i = 0; i < 10; i++) {
    avgX += accXHistory[i];
    avgY += accYHistory[i];
    avgZ += accZHistory[i];
  }
  avgX /= 10;
  avgY /= 10;
  avgZ /= 10;
  
  // Calculate variances
  for (int i = 0; i < 10; i++) {
    sumVarX += sq(accXHistory[i] - avgX);
    sumVarY += sq(accYHistory[i] - avgY);
    sumVarZ += sq(accZHistory[i] - avgZ);
  }
  
  // Calculate total vibration level
  vibrationLevel = sqrt(sumVarX + sumVarY + sumVarZ) / 3.0;
  
  // Check if vibration exceeds threshold
  vibrationDetected = (vibrationLevel > VIBRATION_THRESHOLD);
}
