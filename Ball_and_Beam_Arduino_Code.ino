#include <Servo.h>

// === Ultrasonic Sensor Parameters (HC-SR04) ===
const int trigPin = 7;
const int echoPin = 6;
long duration;
float distance;
const int numSamples = 5;   // moving average samples
float distanceBuffer[numSamples];
int bufferIndex = 0;

// === Servo Setup ===
Servo servo_motor;
int servoMotorPin = 9;

// === PID Control Parameters ===
float desiredPosition = 16.0;  // desired ball position in cm
float Kp = 5.0;     // proportional gain
float Ki = 0.05;    // integral gain
float Kd = 1.0;     // derivative gain

float previousError = 0;
float integral = 0;
float dt = 0.05;     // 50 ms loop time (~20 Hz)
float minControl = 8.0;   // minimum servo movement to overcome friction
float maxControl = 80.0;  // maximum beam tilt (servo degrees)

void setup() {
  Serial.begin(9600);

  servo_motor.attach(servoMotorPin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // initialize distance buffer with desired position
  for (int i = 0; i < numSamples; i++) distanceBuffer[i] = desiredPosition;
}

// === Function to Read Ultrasonic Distance (cm) with Moving Average ===
float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms (~5 m)
  float d = duration / 58.0; // convert to cm

  // handle dead zone and invalid readings
  if (d < 2.0 || d > 400) d = desiredPosition;

  // moving average filter
  distanceBuffer[bufferIndex] = d;
  bufferIndex = (bufferIndex + 1) % numSamples;

  float sum = 0;
  for (int i = 0; i < numSamples; i++) sum += distanceBuffer[i];
  return sum / numSamples;
}

void loop() {
  // --- Read filtered distance ---
  distance = readUltrasonic();

  // --- PID Calculation ---
  float error = desiredPosition - distance;
  integral += error * dt;

  // derivative smoothing to avoid spikes
  float rawDerivative = (error - previousError) / dt;
  float alpha = 0.5;
  float derivative = alpha * rawDerivative + (1 - alpha) * rawDerivative;

  float control = Kp * error + Ki * integral + Kd * derivative;

  // constrain max control for stability
  control = constrain(control, -maxControl, maxControl);

  // apply minimum control to overcome friction if error > 1 cm
  if (abs(error) > 1.0 && abs(control) < minControl) {
    control = (control > 0) ? minControl : -minControl;
  }

  // update previous error
  previousError = error;

  // --- Servo Position ---
  int servoPos = constrain(90 + control, 0, 180); // 90 = neutral
  servo_motor.write(servoPos);

  // --- Debug Serial Output ---
  Serial.print("Distance: "); Serial.print(distance,1);
  Serial.print(" cm | Error: "); Serial.print(error,2);
  Serial.print(" | Control: "); Serial.print(control,2);
  Serial.print(" | Servo Pos: "); Serial.println(servoPos);

  delay(dt*1000);
}
