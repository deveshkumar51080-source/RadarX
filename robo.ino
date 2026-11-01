/*
 * ==============================================================================
 * PROJECT: R.A.D.A.R-X (Robotic Adaptive Detection and Response System)
 * AUTHOR: [Your Name]
 * REPOSITORY: https://github.com/[Your-Username]/RADAR-X
 * DESCRIPTION: 
 * Multi-sensor gesture-controlled robotic platform featuring mmWave human 
 * presence detection, LiDAR/Ultrasonic spatial awareness, and a deterministic 
 * hardware kill-switch for automated collision prevention.
 * ==============================================================================
 */

#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_VL53L0X.h>
#include <NewPing.h>
#include <ld2410.h>

// ==========================================
// PIN CONFIGURATIONS
// ==========================================
// 1. Motor Driver (L298N)
#define ENA 12  // PWM Left
#define IN1 13  // Direction 1 Left
#define IN2 14  // Direction 2 Left
#define ENB 27  // PWM Right
#define IN3 26  // Direction 1 Right
#define IN4 25  // Direction 2 Right

// 2. Spatial Sensors (Ultrasonic & ToF)
#define TRIG_PIN 5
#define ECHO_PIN 18
#define I2C_SDA  21
#define I2C_SCL  22

// 3. Human Presence Radar (HLK-LD2410C)
#define RADAR_DET 19 // Digital Out (High = Presence)
#define RADAR_RX  35 // Remapped UART RX (Internal use)
#define RADAR_TX  2  // Remapped UART TX (Internal use)

// 4. Edge/Proximity IR Sensors
#define IR_NEAR 33
#define IR_FAR  34

// 5. Hardware Safety Relay
#define RELAY_PIN 16 // Emergency motor power cut-off

// 6. Output Indicators
#define LED_RED    17  // Danger State
#define LED_GREEN  4   // Safe State
#define LED_YELLOW 32  // Warning State
#define BUZZER     23  // Acoustic Alert

// ==========================================
// OBJECTS & GLOBALS
// ==========================================
Adafruit_APDS9960 apds;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
NewPing sonar(TRIG_PIN, ECHO_PIN, 200); // 200cm Max Distance
ld2410 radar;

bool tofReady = false;
bool radarReady = false;
bool safetyHalt = false;
int motorSpeed = 160; // Default PWM Speed (0-255)

// ==========================================
// INITIALIZATION
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Remap Serial2 to avoid conflict with Relay (16) and Red LED (17)
  Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX); 
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configure GPIO Modes
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(IR_NEAR, INPUT); pinMode(IR_FAR, INPUT);
  pinMode(RADAR_DET, INPUT);
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_YELLOW, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Engage Default States
  digitalWrite(RELAY_PIN, HIGH); // Power motors
  setAlertState(false, false, true, false); // Initialize in GREEN (Safe) mode

  // Initialize Gesture Sensor
  if (!apds.begin()) {
    Serial.println("[ERROR] APDS9960 Gesture Sensor Failed.");
  } else {
    apds.enableGesture(true);
    apds.enableProximity(true);
    Serial.println("[OK] APDS9960 Gesture Ready.");
  }
  
  // Initialize ToF Sensor
  if (!lox.begin()) {
    Serial.println("[WARN] VL53L0X ToF Sensor Failed or Missing.");
    tofReady = false;
  } else {
    tofReady = true;
    Serial.println("[OK] VL53L0X ToF Ready.");
  }
  
  // Initialize mmWave Radar UART (Failsafe to digital if UART disconnected)
  if (!radar.begin(Serial2)) {
    Serial.println("[WARN] LD2410 Radar UART Failed. Relying on Digital Pin 19.");
    radarReady = false;
  } else {
    radarReady = true;
    Serial.println("[OK] LD2410 Radar UART Ready.");
  }
  
  Serial.println("=== R.A.D.A.R-X SYSTEM ONLINE ===");
}

// ==========================================
// MAIN EXECUTION LOOP
// ==========================================
void loop() {
  // Poll Radar UART stream if available
  if (radarReady) radar.read();

  // Evaluate spatial environment continuously
  checkSafetySystem();

  // Process user input only if environment is safe
  if (!safetyHalt) {
    handleGestures();
  } else {
    stopMoving();
  }
  
  delay(50); // Loop stabilization
}

// ==========================================
// CORE FUNCTIONS
// ==========================================

void checkSafetySystem() {
  // 1. Acoustic Ranging (Handle 0cm dropouts gracefully)
  int distUS = sonar.ping_cm();
  if (distUS == 0) distUS = 999; 

  // 2. Optical Ranging
  bool nearObj = (digitalRead(IR_NEAR) == LOW);
  
  // 3. RF Human Presence
  bool radarObj = false;
  if (radarReady) {
    radarObj = radar.presenceDetected();
  } else {
    radarObj = (digitalRead(RADAR_DET) == HIGH); // Fallback to digital logic
  }

  // 4. LiDAR Ranging
  int distToF = 999;
  if (tofReady) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) distToF = measure.RangeMilliMeter / 10;
  }

  // --- DECISION MATRIX ---
  
  // STATE: DANGER (Breach < 15cm)
  if (distUS < 15 || distToF < 15 || nearObj) {
    safetyHalt = true;
    digitalWrite(RELAY_PIN, LOW); // Hard cut-off motor power
    setAlertState(true, false, false, true); // Red LED + Buzzer
  } 
  // STATE: WARNING (Proximity 15-30cm OR Human Detected)
  else if (distUS < 30 || distToF < 30 || radarObj) {
    safetyHalt = false;
    digitalWrite(RELAY_PIN, HIGH); // Restore motor power
    setAlertState(false, true, false, false); // Yellow LED Only
  } 
  // STATE: SAFE (Clear Path)
  else {
    safetyHalt = false;
    digitalWrite(RELAY_PIN, HIGH);
    setAlertState(false, false, true, false); // Green LED Only
  }
}

void handleGestures() {
  uint8_t gesture = apds.readGesture();
  
  if (gesture != 0) {
    switch (gesture) {
      case APDS9960_UP:    
        Serial.println("CMD: FORWARD"); 
        moveRobot(1, 0, 1, 0); 
        break; 
      case APDS9960_DOWN:  
        Serial.println("CMD: REVERSE"); 
        moveRobot(0, 1, 0, 1); 
        break; 
      case APDS9960_LEFT:  
        Serial.println("CMD: TURN LEFT"); 
        moveRobot(0, 1, 1, 0); 
        break; 
      case APDS9960_RIGHT: 
        Serial.println("CMD: TURN RIGHT"); 
        moveRobot(1, 0, 0, 1); 
        break; 
    }
    delay(300); // Actuation window
    stopMoving();
  }
}

// ==========================================
// ACTUATORS & HARDWARE CONTROL
// ==========================================

void moveRobot(int l1, int l2, int r1, int r2) {
  digitalWrite(IN1, l1); digitalWrite(IN2, l2);
  digitalWrite(IN3, r1); digitalWrite(IN4, r2);
  analogWrite(ENA, motorSpeed); 
  analogWrite(ENB, motorSpeed);
}

void stopMoving() {
  digitalWrite(IN1, 0); digitalWrite(IN2, 0);
  digitalWrite(IN3, 0); digitalWrite(IN4, 0);
  analogWrite(ENA, 0);  analogWrite(ENB, 0);
}

void setAlertState(bool r, bool y, bool g, bool bz) {
  digitalWrite(LED_RED, r);
  digitalWrite(LED_YELLOW, y);
  digitalWrite(LED_GREEN, g);
  if (bz) tone(BUZZER, 1000); else noTone(BUZZER);
}