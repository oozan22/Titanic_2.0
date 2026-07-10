#include <Arduino.h>
#include <Servo.h>
#include <IBusBM.h>
// #include <ServoTimer2.h>
// ---------------- HARDWARE CONSTANTS ----------------
#define SWEEP_WIDTH (180)
#define STEP_ANGLE_SIZE (10)
#define MAX_PULSE_RETURN_us (20000)
#define MIN_PULSE_RETURN_us (50)
#define INITIAL_ANGLE (60)
#define MIN_ANGLES_TO_REACT (10)

// Non-blocking timings (State Machine Tuning)
#define SONAR_STEP_INTERVAL_MS (100)  // Safe delay between physical servo steps
#define SERVO_SETTLE_DELAY_MS (100)   // Delay allowed for physical shaft settling

// Pin allocations
const int trigPin = 3;
const int echoPin = 2; // Interrupt pin 
const int servoPin = 4; 

const int right_motor_r_pin = 12;
const int right_motor_l_pin = 13;
const int left_motor_r_pin = 7;
const int left_motor_l_pin = 8;
const int right_motor_pwm_pin = 6;
const int left_motor_pwm_pin = 5;

// ---------------- MODULES & OBJECTS ----------------
// ServoTimer2 sonarServo;
Servo sonarServo;
IBusBM ibusRc;
HardwareSerial& ibusRcSerial = Serial;

// ---------------- STATE MACHINE DEFINITIONS ----------------
enum AutonomousState {
  STATE_SERVO_STEP,
  STATE_TRIGGER_PING,
  STATE_AWAIT_SAMPLE
};

AutonomousState autoState = STATE_SERVO_STEP;

// ---------------- CONTEXT AND MEMORY ----------------
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool newPulseAvailable = false;

constexpr int angles_size = (SWEEP_WIDTH / STEP_ANGLE_SIZE) + 1;
float distances_per_angles[angles_size];

int currentAngle_index = 0;
int sampleAngle_index = 0;
int step_angle_index_signed = 1; // 1 = Forward index loop, -1 = Backward loop

unsigned long last_sonar_step_millis = 0;
unsigned long servo_move_timestamp = 0;

float angle_max = INITIAL_ANGLE;

// ---------------- SYSTEM PROTOTYPES ----------------
void echoISR();
int argmax(const float* arr, int len);
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
void control_ship_instantaneous(int r, int l);

// ---------------- CORE ARCHITECTURE ----------------

void setup() {
  // Initialize communication interfaces
  ibusRcSerial.begin(115200); 
  // ibusRc.begin(ibusRcSerial);
  ibusRc.begin(ibusRcSerial, IBUSBM_NOTIMER);
  Serial.begin(115200);

  // Configure Hardware I/O Peripheral Modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(right_motor_l_pin, OUTPUT);
  pinMode(right_motor_r_pin, OUTPUT);
  pinMode(left_motor_r_pin, OUTPUT);
  pinMode(left_motor_l_pin, OUTPUT);
  pinMode(left_motor_pwm_pin, OUTPUT);
  pinMode(right_motor_pwm_pin, OUTPUT);

  // Initialize distance mapping registry to prevent processing unallocated garbage memory
  for (int i = 0; i < angles_size; i++) {
    distances_per_angles[i] = 3.43f; // Default baseline to clear obstacles
  }

  // Bind low-latency change state interrupt to echo pin
  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

  // Home sonar array assembly safely
  sonarServo.attach(servoPin);
  delay(10);
  sonarServo.write(INITIAL_ANGLE);
  delay(50); 
}

void loop() {
  ibusRc.loop();
  // Sync Nav Trigger condition at the start of loop execution to fix input-lag bugs
  int nav_trig = -readChannel(9, 0, 1, 0) + 1;
  int r = 0;
  int l = 0;
  // ================= BRANCH A: AUTONOMOUS NAVIGATION =================
  if (nav_trig == 0) { 
    unsigned long currentMillis = millis();

    switch (autoState) {
      case STATE_SERVO_STEP:
        if (currentMillis - last_sonar_step_millis >= SONAR_STEP_INTERVAL_MS) {
          last_sonar_step_millis = currentMillis;

          // Compute max environmental range data vector when completing sweeping thresholds
          if (currentAngle_index == 0 || currentAngle_index >= angles_size - 1) { 
            angle_max = (float)argmax(distances_per_angles, angles_size) * STEP_ANGLE_SIZE + INITIAL_ANGLE;
            Serial.print("Target Vector Sweep Max Angle: ");
            Serial.println(angle_max);
          }

          sampleAngle_index = currentAngle_index;
          float targetAngle = (currentAngle_index * STEP_ANGLE_SIZE) + INITIAL_ANGLE;
          sonarServo.write(constrain(targetAngle, 0, 180));
          
          servo_move_timestamp = currentMillis;
          autoState = STATE_TRIGGER_PING;
        }
        break;

      case STATE_TRIGGER_PING:
        // Confirm mechanical servo assembly has finished slewing to angle before pinging
        if (currentMillis - servo_move_timestamp >= SERVO_SETTLE_DELAY_MS) {
          digitalWrite(trigPin, LOW);
          delayMicroseconds(2);
          digitalWrite(trigPin, HIGH);
          delayMicroseconds(10);
          digitalWrite(trigPin, LOW);

          // Queue indexing stepping configurations
          currentAngle_index += step_angle_index_signed;
          if (currentAngle_index >= angles_size - 1) {
            step_angle_index_signed = -1;
          } else if (currentAngle_index <= 0) {
            step_angle_index_signed = 1;
          }
          
          autoState = STATE_AWAIT_SAMPLE;
        }
        break;

      case STATE_AWAIT_SAMPLE:
        // Transition immediately when data returns or drop state context if waiting times out
        if (newPulseAvailable || (currentMillis - last_sonar_step_millis > SONAR_STEP_INTERVAL_MS)) {
          autoState = STATE_SERVO_STEP;
        }
        break;
    }

    // Safely parse interrupt timing frames using an atomic copy operation block
    if (newPulseAvailable) {
      noInterrupts();
      unsigned long start = echoStart;
      unsigned long end = echoEnd;
      newPulseAvailable = false;
      interrupts();

      unsigned long pulseDurationUs = end - start;
      float sample_distance = 3.43f;

      if (pulseDurationUs < MAX_PULSE_RETURN_us && pulseDurationUs > MIN_PULSE_RETURN_us) {
        sample_distance = ((float)pulseDurationUs / 1000000.0f) * 343.0f / 2.0f;
      }
      distances_per_angles[sampleAngle_index] = sample_distance;
    }

    // Motor Speed Control Configurations - TODO: REFINE
    int speed = 128; // Configured drive speed -
    float angle_max_normalized = angle_max - INITIAL_ANGLE - ((float)SWEEP_WIDTH / 2.0f);
    int raw_dir = 0;
    if (angle_max_normalized < -MIN_ANGLES_TO_REACT || angle_max_normalized > MIN_ANGLES_TO_REACT) {
      raw_dir = map((long)(angle_max_normalized), -90, 90, -speed, speed);
    }
    if(raw_dir<0){
      r = speed;
      l = speed - raw_dir;
    }
    else{
      r = speed - raw_dir;
      l = speed;
    }

  } 
  // ================= BRANCH B: MANUAL REMOTE CONTROL =================
  else { 

    int control_speed = readChannel(2, 0, 256, 0);
    int gear_shift = -readChannel(8, 0, 2, 0) + 3;
    int speed_mode = -readChannel(6, 0, 2, 0) + 1;
    int speed = speed_mode * (int)((float)control_speed * ((float)gear_shift / 3.0f));
    int raw_dir = readChannel(0, -speed, speed, 0) ;
    
    if(raw_dir<0){
      r = speed;
      l = speed - raw_dir;
    }
    else{
      r = speed - raw_dir;
      l = speed;
    }

  }

  // Pass sanitized values down to propulsion system arrays
  control_ship_instantaneous(r,l);
}

// ---------------- HARDWARE PERIPHERALS & UTILS ----------------

void echoISR() {
  if (digitalRead(echoPin) == HIGH) {
    echoStart = micros();
  } else {
    echoEnd = micros();
    newPulseAvailable = true;
  }
}

int argmax(const float* arr, int len) {
  int max_ind = 0;
  float saved_max = arr[0];
  for (int i = 1; i < len; i++) {
    if (arr[i] > saved_max) {
      saved_max = arr[i];
      max_ind = i;
    }
  }
  return max_ind;
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue; // Active signal line protection loop
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void control_ship_instantaneous(int r, int l) {

  //check input.
  r = constrain(r, -255, 255);
  l = constrain(l, -255, 255);

  analogWrite(right_motor_pwm_pin, abs(r));
  if (r<0) {
    digitalWrite(right_motor_r_pin, LOW);
    digitalWrite(right_motor_l_pin, HIGH);
  } else {
    digitalWrite(right_motor_r_pin, HIGH);
    digitalWrite(right_motor_l_pin, LOW);
  }

  analogWrite(left_motor_pwm_pin, abs(l));
  if (l>0) {
    digitalWrite(left_motor_r_pin, LOW);
    digitalWrite(left_motor_l_pin, HIGH);
  } else {
    digitalWrite(left_motor_l_pin, LOW);
    digitalWrite(left_motor_r_pin, HIGH);
  }
}