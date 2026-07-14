#include <Arduino.h>
#include <Servo.h>
#include <IBusBM.h>
// #include <ServoTimer2.h>
// ---------------- HARDWARE CONSTANTS ----------------
#define SWEEP_WIDTH (90)
#define STEP_ANGLE_SIZE (15)
#define MAX_PULSE_RETURN_us (20000)
#define MIN_PULSE_RETURN_us (100)
#define INITIAL_ANGLE (0)
#define MIN_ANGLES_TO_REACT (10)

#define BACK_SERVO_SWEEP_WIDTH (140)
#define BACK_SERVO_INIT_ANGLE (95)

// Non-blocking timings (State Machine Tuning)
#define SONAR_STEP_INTERVAL_MS (200)  // Safe delay between physical servo steps
#define SERVO_SETTLE_DELAY_MS (100)   // Delay allowed for physical shaft settling

#define PUMP_DURATION_MS (2000)

// Pin allocations
const int trigPin = 3;
const int echoPin = 2; // Interrupt pin 
const int sonar_servoPin = 4; 

const int jet_fwd_pin1 = 7;
const int jet_rev_pin1 = 8;
const int jet_fwd_pin2 = 12;
const int jet_rev_pin2 = 13;

const int jet_pwm_pin1 = 5;
const int jet_pwm_pin2 = 6;

const int jet_dir_servo_pin = 9;

const int pump_pin = 10;
const int rescuse_servo_pin = 11;

// ---------------- MODULES & OBJECTS ----------------
// ServoTimer2 sonarServo;
Servo sonarServo;
Servo jetServo;
Servo rescuseServo;
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
unsigned long pump_start_time = 0;

float angle_max = INITIAL_ANGLE;

int pump_flag = 1;
// ---------------- SYSTEM PROTOTYPES ----------------
void echoISR();
int argmax(const float* arr, int len,int def_val);
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
void control_ship_instantaneous_jet(int speed0,int dir0);
void print_arr(float* arr, int len);
void conv_same(float* arr1,float* arr2,int len1,int len2,float* conved_arr_same);
// ---------------- CORE ARCHITECTURE ----------------

void setup() {

  ibusRcSerial.begin(115200); 
  ibusRc.begin(ibusRcSerial, IBUSBM_NOTIMER);
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(jet_fwd_pin1, OUTPUT);
  pinMode(jet_rev_pin1, OUTPUT);
  pinMode(jet_pwm_pin1, OUTPUT);

  pinMode(jet_fwd_pin2, OUTPUT);
  pinMode(jet_rev_pin2, OUTPUT);
  pinMode(jet_pwm_pin2, OUTPUT);

  pinMode(pump_pin, OUTPUT);


  // Initialize distance mapping 
  for (int i = 0; i < angles_size; i++) {
    distances_per_angles[i] = 0; // Default baseline to clear obstacles
  }

  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

  sonarServo.attach(sonar_servoPin);
  delay(10);
  sonarServo.write(INITIAL_ANGLE);
  delay(50); 

  jetServo.attach(jet_dir_servo_pin);
  delay(10);
  jetServo.write(BACK_SERVO_INIT_ANGLE);
  delay(50); 

  rescuseServo.attach(rescuse_servo_pin);
  delay(10);
  rescuseServo.write(0);
  delay(50);
}

void loop() {
  ibusRc.loop();
  // Sync Nav Trigger condition at the start of loop execution to fix input-lag bugs
  int nav_trig = -readChannel(9, 0, 1, 0) + 1;

  int jet_speed = 0;
  int jet_dir = 0;
  // ================= BRANCH A: AUTONOMOUS NAVIGATION =================
  if (nav_trig == 0) { 
    unsigned long currentMillis = millis();

    switch (autoState) {
      case STATE_SERVO_STEP:
        if (currentMillis - last_sonar_step_millis >= SONAR_STEP_INTERVAL_MS) {
          last_sonar_step_millis = currentMillis;

          // Compute max environmental range data vector when completing sweeping thresholds
          if (currentAngle_index == 0 || currentAngle_index >= angles_size - 1) { 
            float window[3] = {1.0f/3.0f, 1.0f/3.0f, 1.0f/3.0f};
            
            // Create the destination array here
            float conved_arr_same[angles_size];
            
            // Pass the destination array directly into the function
            conv_same(distances_per_angles, window, angles_size, 3, conved_arr_same);
            
            //def_val : middle angle
            angle_max = (float)argmax(conved_arr_same, angles_size, (int)(angles_size/2)) * STEP_ANGLE_SIZE + INITIAL_ANGLE;
            Serial.print("Target Vector Sweep Max Angle: ");
            Serial.println(angle_max);

            Serial.println("angles_arr:");
            print_arr(distances_per_angles, angles_size);

            Serial.println("angles_conved:");
            print_arr(conved_arr_same, angles_size);
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
      float sample_distance = 0.0f;

      if (pulseDurationUs < MAX_PULSE_RETURN_us && pulseDurationUs > MIN_PULSE_RETURN_us) {
        sample_distance = ((float)pulseDurationUs / 1000000.0f) * 343.0f / 2.0f;
      }
      distances_per_angles[sampleAngle_index] = sample_distance;
    }

    // Motor Speed Control Configurations - TODO: REFINE
    int speed = 120; // Configured drive speed -
    float angle_max_normalized = angle_max - INITIAL_ANGLE - ((float)SWEEP_WIDTH / 2.0f);
    int raw_dir = 0;
    if (angle_max_normalized < -MIN_ANGLES_TO_REACT || angle_max_normalized > MIN_ANGLES_TO_REACT) {
      raw_dir = (long)(angle_max_normalized);
    }
    if (raw_dir!=0){
      jet_speed = constrain(abs(speed/raw_dir)*200, 0, 255);
    }
    else{
      jet_speed = speed;

    }
    jet_dir = -raw_dir;


  } 
  // ================= BRANCH B: MANUAL REMOTE CONTROL =================
  else { 

    int control_speed = readChannel(2, 0, 256, 0);
    int gear_shift = -readChannel(8, 0, 2, 0) + 3;
    int speed_mode = -readChannel(6, 0, 2, 0) + 1;
    int speed = speed_mode * (int)((float)control_speed * ((float)gear_shift / 3.0f));
    int raw_dir = readChannel(0, -BACK_SERVO_SWEEP_WIDTH/2, BACK_SERVO_SWEEP_WIDTH/2, 0);
    jet_speed = speed;
    jet_dir = raw_dir;
  }

 //control ship
  
  control_ship_instantaneous_jet(jet_speed,jet_dir);

  int resc_raise_angle = readChannel(5, 0, 30, 0);
  int pump_on = readChannel(4, 0, 1, 0);

  rescuseServo.write(resc_raise_angle);
  if (pump_on==1 && pump_flag==1) {
    digitalWrite(pump_pin, HIGH);
    pump_start_time = millis();
    pump_flag = 0;
  }
  if (pump_flag == 0 && millis() - pump_start_time > PUMP_DURATION_MS) {
    digitalWrite(pump_pin, LOW);
  }
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

int argmax(const float* arr, int len,int def_val) {
  int max_ind = -1;
  float saved_max = arr[0];
  for (int i = 0; i < len; i++) {
    if (arr[i] > saved_max) {
      saved_max = arr[i];
      max_ind = i;
    }
  }
  if(max_ind==-1){
    return def_val;
  }
  
  return max_ind;
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue; // Active signal line protection loop
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


void control_ship_instantaneous_jet(int speed0,int dir0){
  
  // Constrain parameters to standard 8-bit dynamic resolution range
  speed0 = constrain(speed0, -255, 255);
  // dir0 = constrain(dir0, -255, 255);

  analogWrite(jet_pwm_pin1, abs(speed0));
  if (speed0>0) {
    digitalWrite(jet_fwd_pin1, HIGH);
    digitalWrite(jet_rev_pin1, LOW);
  } else {
    digitalWrite(jet_fwd_pin1, LOW);
    digitalWrite(jet_rev_pin1, HIGH);
  }

  analogWrite(jet_pwm_pin2, abs(speed0));
  if (speed0>0) {
    digitalWrite(jet_fwd_pin2, HIGH);
    digitalWrite(jet_rev_pin2, LOW);
  } else {
    digitalWrite(jet_fwd_pin2, LOW);
    digitalWrite(jet_rev_pin2, HIGH);
  }


  if (dir0!=0) {
    jetServo.write(BACK_SERVO_INIT_ANGLE+dir0);
  }
}
void print_arr(float* arr, int len) {
  Serial.print(F("[")); // Safe storage in Flash
  for (int i = 0; i < len; i++) {
    Serial.print(arr[i]);
    if (i < len - 1) {
      Serial.print(F(", ")); // Safe storage in Flash
    }
  }
  Serial.println(F("]"));
}

void conv_same(float* arr1, float* arr2, int len1, int len2, float* output) {
    // 1. Explicitly clear the output array to zero to remove garbage data
    for (int i = 0; i < len1; i++) {
        output[i] = 0.0f;
    }

    for (int i = 0; i < len1; ++i) {
        for (int j = 0; j < len2; ++j) {
            // Keep indices strictly within the output slice bounds (len1)
            if (i + j < len1) { 
                output[i + j] += arr1[i] * arr2[j];
            }
        }
    }
}