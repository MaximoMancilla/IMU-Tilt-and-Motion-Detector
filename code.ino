/*******************************************************
 * ESP32 + MPU6050 Tilt and Jolt Detection
 *
 * Functionality:
 * - 4 LEDs indicate tilt in 4 directions (forward, backward, left, right)
 * - Buzzer sounds when acceleration exceeds a threshold (detects jolts)
 *
 * Hardware:
 * - ESP32
 * - MPU6050 accelerometer + gyroscope
 * - 4 LEDs with current-limiting resistors
 * - Piezo buzzer
 *
 * Author: Maximo Mancilla
 *******************************************************/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h> // atan2() and sqrt() for angle calculations

// Create MPU6050 object
Adafruit_MPU6050 mpu;

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// LED pins
#define LED_FORWARD  32
#define LED_BACKWARD 33
#define LED_RIGHT    25
#define LED_LEFT     26

// Buzzer pin
#define BUZZER_PIN   15

// Thresholds
const float TILT_THRESHOLD = 10.0; // degrees; LED triggers beyond this tilt
const float ACC_THRESHOLD  = 12.0; // m/s²; buzzer triggers beyond this acceleration (jolts)

void setup() {
  Serial.begin(115200); // Initialize serial monitor for debugging
  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C

  // Set pins as outputs
  pinMode(LED_FORWARD, OUTPUT);
  pinMode(LED_BACKWARD, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10); // Stop execution if sensor not found
  }

  // Configure MPU6050 ranges and filter
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);   // ±4g, suitable for desktop motions
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // ±250°/s, for slow tilts
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Low-pass filter to reduce jitter

  delay(100); // Allow sensor to stabilize
}

void loop() {
  // Create sensor event objects
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Read accelerometer, gyro, and temperature 
  // =========================
  // Calculate tilt angles
  // =========================
  // Roll: rotation around X-axis (left/right tilt)
  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  // Pitch: rotation around Y-axis (forward/backward tilt)
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0 / PI;

  // =========================
  // Reset LEDs
  // =========================
  digitalWrite(LED_FORWARD, LOW);
  digitalWrite(LED_BACKWARD, LOW);
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);

  // =========================
  // LED Logic: Tilt Detection
  // =========================
  // Forward/backward LEDs based on pitch
  if (pitch > TILT_THRESHOLD) {
    digitalWrite(LED_RIGHT, HIGH);
  } else if (pitch < -TILT_THRESHOLD) {
    digitalWrite(LED_LEFT, HIGH);
  }

  // Left/right LEDs based on roll
  if (roll > TILT_THRESHOLD) {
    digitalWrite(LED_BACKWARD, HIGH);
  } else if (roll < -TILT_THRESHOLD) {
    digitalWrite(LED_FORWARD, HIGH);
  }

  // =========================
  // Buzzer Logic: Jolt Detection
  // =========================
  // Calculate total acceleration magnitude (combines X, Y, Z)
  float acc_magnitude = sqrt(a.acceleration.x*a.acceleration.x +
                       a.acceleration.y*a.acceleration.y +
                       a.acceleration.z*a.acceleration.z);

  // Trigger buzzer if acceleration exceeds threshold
  if (acc_magnitude > ACC_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // =========================
  // Debugging output
  // =========================
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print("Acceleration magnitude: "); Serial.println(acc_magnitude);

  delay(100); // 100ms loop delay for smooth response
}
