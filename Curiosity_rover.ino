/*
 * AI Curiosity Rover Bot - WiFi + UDP Control (INTELLIGENT AUTO MODE)
 * Enhanced with speed-adaptive turning and dead-end detection
 * FIXED: Compilation errors resolved
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>

WebServer server(80);
WiFiUDP udp;

const char* ap_ssid = "Curiosity";
const char* ap_password = "rover12345";
const unsigned int udpPort = 1234;

// --- Pin Definitions ---
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 12

#define TRIG_PIN 4
#define ECHO_PIN 2

// --- PWM Constants ---
const int ENA_CHANNEL = 0;
const int ENB_CHANNEL = 1;
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

// --- Sensor Constants ---
#define MAX_SENSOR_DIST 400
#define SENSOR_PING_INTERVAL 50
#define DIST_INVALID 999

// --- Navigation & Safety ---
#define SAFE_DISTANCE 20
#define CRITICAL_DISTANCE 15
#define OPTIMAL_DISTANCE 40
#define MIN_CLEARANCE 15
#define STATUS_PRINT_INTERVAL 3000

// --- Dead End Detection ---
#define DEAD_END_THRESHOLD 4  // Number of consecutive failed attempts
#define DEAD_END_MIN_TIME 3000  // Minimum time (ms) before considering dead end
unsigned long deadEndStartTime = 0;
int failedTurnAttempts = 0;
bool inDeadEndRecovery = false;

// --- Motor Speed ---
int defaultSpeed = 160;
int turnSpeed = 150;
const int maxSpeed = 200;
const int minSpeed = 100;
const int REFERENCE_SPEED = 150;  // Reference speed for timing calibration

// --- Base Intelligent Navigation Timings (calibrated at REFERENCE_SPEED) ---
const int BASE_QUICK_SCAN_TIME = 300;
const int BASE_TURN_90_TIME = 700;
const int BASE_TURN_45_TIME = 400;
const int BASE_BACKUP_SHORT = 400;
const int BASE_BACKUP_LONG = 800;
const int BASE_FORWARD_BURST = 1500;

// --- Dynamic timing variables (will be calculated based on current speed) ---
int QUICK_SCAN_TIME = BASE_QUICK_SCAN_TIME;
int TURN_90_TIME = BASE_TURN_90_TIME;
int TURN_45_TIME = BASE_TURN_45_TIME;
int BACKUP_SHORT = BASE_BACKUP_SHORT;
int BACKUP_LONG = BASE_BACKUP_LONG;
int FORWARD_BURST = BASE_FORWARD_BURST;

// --- Intelligence Constants ---
const int EXPLORATION_HISTORY = 5;
const int RANDOM_TURN_THRESHOLD = 3;
const unsigned long FORWARD_MIN_TIME = 2000;

// --- Command Buffer ---
#define RX_BUFFER_SIZE 64
const char* SINGLE_CHAR_CMDS = "FBLRXMA?S";

// --- States ---
enum OpMode { MODE_MANUAL, MODE_AUTO };
OpMode currentMode = MODE_MANUAL;
OpMode previousMode = MODE_MANUAL;

enum MotorState { STATE_STOPPED, STATE_FORWARD, STATE_BACK, STATE_LEFT, STATE_RIGHT };
MotorState currentMotorState = STATE_STOPPED;

// --- Intelligent Auto Navigation States ---
enum AutoState {
  AUTO_MOVING_FORWARD,
  AUTO_OBSTACLE_DETECTED,
  AUTO_SCANNING,
  AUTO_DECIDING,
  AUTO_TURNING,
  AUTO_BACKING_UP,
  AUTO_STUCK_RECOVERY,
  AUTO_DEAD_END_RECOVERY
};
AutoState autoState = AUTO_MOVING_FORWARD;

// --- Sensor Variables ---
long currentDistance = DIST_INVALID;
unsigned long lastSensorUpdate = 0;

// --- Auto Navigation Variables ---
unsigned long autoStateStartTime = 0;
int turnDirection = 0;
int turnAmount = 0;
bool needsBackup = false;

// Scan results
int leftDistance = DIST_INVALID;
int leftFarDistance = DIST_INVALID;
int centerDistance = DIST_INVALID;
int rightDistance = DIST_INVALID;
int rightFarDistance = DIST_INVALID;

// Intelligence tracking
int directionHistory[EXPLORATION_HISTORY] = {0};
int historyIndex = 0;
int consecutiveStuckCount = 0;
unsigned long lastDirectionChangeTime = 0;
unsigned long lastForwardStartTime = 0;
int lastTurnDirection = 0;

// --- Globals ---
unsigned long lastStatusPrint = 0;
bool emergencyStop = false;

struct CommandBuffer {
  char buffer[RX_BUFFER_SIZE + 1];
  int index = 0;
};

CommandBuffer serialBuffer;

// ==================== HELPER FUNCTIONS ====================

unsigned long timeDiff(unsigned long now, unsigned long then) {
  if (now >= then) {
    return now - then;
  } else {
    return (ULONG_MAX - then) + now + 1;
  }
}

// Calculate speed scaling factor for timing adjustments
float getSpeedScaleFactor() {
  // Turn time should be inversely proportional to speed
  // If speed is lower, we need MORE time to achieve the same turn angle
  float factor = (float)REFERENCE_SPEED / (float)turnSpeed;
  
  // Add a slight bonus multiplier for very low speeds to ensure full turns
  if (turnSpeed < 120) {
    factor *= 1.15;  // 15% extra time for very low speeds
  }
  
  return constrain(factor, 0.5, 2.5);  // Limit the scaling range
}

// Update all timing constants based on current speed
void updateTimingConstants() {
  float scaleFactor = getSpeedScaleFactor();
  
  TURN_90_TIME = (int)(BASE_TURN_90_TIME * scaleFactor);
  TURN_45_TIME = (int)(BASE_TURN_45_TIME * scaleFactor);
  BACKUP_SHORT = (int)(BASE_BACKUP_SHORT * scaleFactor);
  BACKUP_LONG = (int)(BASE_BACKUP_LONG * scaleFactor);
  QUICK_SCAN_TIME = (int)(BASE_QUICK_SCAN_TIME * scaleFactor);
  
  // Forward burst timing doesn't scale with turn speed
  FORWARD_BURST = BASE_FORWARD_BURST;
  
  Serial.print("⚙️ Timing adjusted - Scale: ");
  Serial.print(scaleFactor, 2);
  Serial.print(" | 90°: ");
  Serial.print(TURN_90_TIME);
  Serial.print("ms | 45°: ");
  Serial.print(TURN_45_TIME);
  Serial.println("ms");
}

void addToHistory(int direction) {
  directionHistory[historyIndex] = direction;
  historyIndex = (historyIndex + 1) % EXPLORATION_HISTORY;
}

bool isRepeatingPattern() {
  int leftCount = 0, rightCount = 0;
  for (int i = 0; i < EXPLORATION_HISTORY; i++) {
    if (directionHistory[i] == -1) leftCount++;
    if (directionHistory[i] == 1) rightCount++;
  }
  return (leftCount > 2 && rightCount > 2);
}

// Dead End Detection Logic
void checkDeadEndCondition() {
  // Check if we're stuck in the same area with all paths blocked
  if (centerDistance != DIST_INVALID && centerDistance < CRITICAL_DISTANCE &&
      leftDistance != DIST_INVALID && leftDistance < CRITICAL_DISTANCE &&
      rightDistance != DIST_INVALID && rightDistance < CRITICAL_DISTANCE) {
    
    if (deadEndStartTime == 0) {
      deadEndStartTime = millis();
      failedTurnAttempts = 0;
    }
    
    failedTurnAttempts++;
    
    // If we've been stuck for minimum time and have enough failed attempts
    if (timeDiff(millis(), deadEndStartTime) > DEAD_END_MIN_TIME && 
        failedTurnAttempts >= DEAD_END_THRESHOLD) {
      
      Serial.println("🚧🚧🚧 DEAD END DETECTED! Initiating 180° escape 🚧🚧🚧");
      inDeadEndRecovery = true;
      autoState = AUTO_DEAD_END_RECOVERY;
      deadEndStartTime = 0;
      failedTurnAttempts = 0;
    }
  } else {
    // Reset dead end tracking if we have some clear space
    if (centerDistance > SAFE_DISTANCE || leftDistance > SAFE_DISTANCE || rightDistance > SAFE_DISTANCE) {
      deadEndStartTime = 0;
      failedTurnAttempts = 0;
    }
  }
}

// ==================== MOTOR CONTROL ====================

void setMotorPWM(int speedA, int speedB) {
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(ENA, speedA);
    ledcWrite(ENB, speedB);
  #else
    ledcWrite(ENA_CHANNEL, speedA);
    ledcWrite(ENB_CHANNEL, speedB);
  #endif
}

void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(ENA_CHANNEL, PWM_FREQ, PWM_RES);
    ledcSetup(ENB_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, ENA_CHANNEL);
    ledcAttachPin(ENB, ENB_CHANNEL);
  #endif

  setMotorPWM(0, 0);
  goStop();
  Serial.println("✅ Motors Ready");
}

void goForward() {
  if (emergencyStop) return;
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorPWM(defaultSpeed, defaultSpeed);
  
  if (currentMotorState != STATE_FORWARD) {
    currentMotorState = STATE_FORWARD;
  }
}

void goBack() {
  if (emergencyStop) return;
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorPWM(defaultSpeed, defaultSpeed);
  
  if (currentMotorState != STATE_BACK) {
    currentMotorState = STATE_BACK;
  }
}

void goLeft() {
  if (emergencyStop) return;
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorPWM(turnSpeed, turnSpeed);
  
  if (currentMotorState != STATE_LEFT) {
    currentMotorState = STATE_LEFT;
  }
}

void goRight() {
  if (emergencyStop) return;
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorPWM(turnSpeed, turnSpeed);
  
  if (currentMotorState != STATE_RIGHT) {
    currentMotorState = STATE_RIGHT;
  }
}

void goStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setMotorPWM(0, 0);
  
  if (currentMotorState != STATE_STOPPED) {
    currentMotorState = STATE_STOPPED;
  }
}

// ==================== ULTRASONIC SENSOR ====================

void setupSensor() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delay(50);
  Serial.println("✅ Sensor Ready");
}

long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration == 0) {
    return DIST_INVALID;
  }
  
  long distance = duration * 0.034 / 2;
  
  if (distance > 0 && distance <= MAX_SENSOR_DIST) {
    return distance;
  }
  
  return DIST_INVALID;
}

void updateSensor() {
  unsigned long currentTime = millis();
  
  if (timeDiff(currentTime, lastSensorUpdate) >= SENSOR_PING_INTERVAL) {
    currentDistance = readDistance();
    lastSensorUpdate = currentTime;
  }
}

long getStableDistance(int samples = 3) {
  long validReadings[5];
  int validCount = 0;
  
  for (int i = 0; i < samples && validCount < 3; i++) {
    long dist = readDistance();
    if (dist != DIST_INVALID) {
      validReadings[validCount++] = dist;
    }
    delay(20);
  }
  
  if (validCount == 0) return DIST_INVALID;
  
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (validReadings[i] > validReadings[j]) {
        long temp = validReadings[i];
        validReadings[i] = validReadings[j];
        validReadings[j] = temp;
      }
    }
  }
  
  return validReadings[validCount / 2];
}

// ==================== INTELLIGENT SCANNING ====================

void performIntelligentScan() {
  Serial.println("🔍 Intelligent 5-point scan...");
  
  // Update timing constants before scanning
  updateTimingConstants();
  
  // Center reading
  goStop();
  delay(150);
  centerDistance = getStableDistance(3);
  Serial.print("   Center: ");
  Serial.println(centerDistance);
  
  // Right scan (45 degrees)
  goRight();
  delay(TURN_45_TIME);
  goStop();
  delay(100);
  rightDistance = getStableDistance(2);
  Serial.print("   Right 45°: ");
  Serial.println(rightDistance);
  
  // Far right (90 degrees total)
  goRight();
  delay(TURN_45_TIME);
  goStop();
  delay(100);
  rightFarDistance = getStableDistance(2);
  Serial.print("   Right 90°: ");
  Serial.println(rightFarDistance);
  
  // Return to center and go left
  goLeft();
  delay(TURN_90_TIME);
  goStop();
  delay(100);
  
  // Left scan (45 degrees)
  goLeft();
  delay(TURN_45_TIME);
  goStop();
  delay(100);
  leftDistance = getStableDistance(2);
  Serial.print("   Left 45°: ");
  Serial.println(leftDistance);
  
  // Far left (90 degrees total)
  goLeft();
  delay(TURN_45_TIME);
  goStop();
  delay(100);
  leftFarDistance = getStableDistance(2);
  Serial.print("   Left 90°: ");
  Serial.println(leftFarDistance);
  
  // Return to center
  goRight();
  delay(TURN_90_TIME);
  goStop();
  delay(100);
}

// ==================== INTELLIGENT DECISION MAKING ====================

void makeIntelligentDecision() {
  Serial.println("🧠 Analyzing paths...");

  int centerScore = (centerDistance != DIST_INVALID && centerDistance > MIN_CLEARANCE) ? centerDistance : 0;
  int leftScore = (leftDistance != DIST_INVALID && leftDistance > MIN_CLEARANCE) ? leftDistance : 0;
  int leftFarScore = (leftFarDistance != DIST_INVALID && leftFarDistance > MIN_CLEARANCE) ? leftFarDistance : 0;
  int rightScore = (rightDistance != DIST_INVALID && rightDistance > MIN_CLEARANCE) ? rightDistance : 0;
  int rightFarScore = (rightFarDistance != DIST_INVALID && rightFarDistance > MIN_CLEARANCE) ? rightFarDistance : 0;

  int bestLeftPath = max(leftScore, leftFarScore);
  int bestRightPath = max(rightScore, rightFarScore);

  if (centerScore > OPTIMAL_DISTANCE) centerScore += 30;
  if (bestLeftPath > OPTIMAL_DISTANCE) bestLeftPath += 20;
  if (bestRightPath > OPTIMAL_DISTANCE) bestRightPath += 20;

  if (lastTurnDirection == -1) {
    bestLeftPath *= 0.7;
  } else if (lastTurnDirection == 1) {
    bestRightPath *= 0.7;
  }

  Serial.print("📊 Scores - Center:");
  Serial.print(centerScore);
  Serial.print(" | Best Left:");
  Serial.print(bestLeftPath);
  Serial.print(" | Best Right:");
  Serial.println(bestRightPath);

  // Check for dead end condition
  checkDeadEndCondition();

  needsBackup = false;

  if (isRepeatingPattern() && consecutiveStuckCount >= RANDOM_TURN_THRESHOLD) {
    Serial.println("🎲 Breaking pattern with random turn");
    turnDirection = (random(0, 2) == 0) ? -1 : 1;
    turnAmount = TURN_90_TIME + random(0, TURN_90_TIME / 2);
    needsBackup = true;
    consecutiveStuckCount = 0;
    return;
  }

  if (centerScore == 0 && bestLeftPath == 0 && bestRightPath == 0) {
    Serial.println("🚧 All paths blocked - 180° turn");
    turnDirection = (lastTurnDirection == -1) ? 1 : -1;
    turnAmount = TURN_90_TIME * 2;
    needsBackup = true;
    consecutiveStuckCount++;
  }
  else if (centerScore >= bestLeftPath && centerScore >= bestRightPath && centerScore > 0) {
    Serial.println("✅ Center clear - Going straight");
    turnDirection = 0;
    turnAmount = 0;
    consecutiveStuckCount = 0;
  }
  else if (bestLeftPath > centerScore && bestLeftPath > bestRightPath) {
    Serial.println("↪️  Left path is best");
    turnDirection = -1;

    if (leftScore > (leftFarScore + 20)) { 
      turnAmount = TURN_45_TIME;
      Serial.println("    Gentle left turn (45°)");
    } else {
      turnAmount = TURN_90_TIME;
      Serial.println("    Standard left turn (90°)");
    }
    consecutiveStuckCount = 0;
  }
  else {
    Serial.println("↩️  Right path is best");
    turnDirection = 1;

    if (rightScore > (rightFarScore + 20)) { 
      turnAmount = TURN_45_TIME;
      Serial.println("    Gentle right turn (45°)");
    } else {
      turnAmount = TURN_90_TIME;
      Serial.println("    Standard right turn (90°)");
    }
    consecutiveStuckCount = 0;
  }

  if (!needsBackup) {
    if (turnDirection == -1 && bestLeftPath < SAFE_DISTANCE) {
      Serial.println("    Path is tight, backing up first.");
      needsBackup = true;
    } else if (turnDirection == 1 && bestRightPath < SAFE_DISTANCE) {
      Serial.println("    Path is tight, backing up first.");
      needsBackup = true;
    } else if (turnDirection == 0 && centerScore < SAFE_DISTANCE) {
      Serial.println("    Center path is tight, backing up first.");
      needsBackup = true;
      turnDirection = (lastTurnDirection == -1) ? 1 : -1;
      turnAmount = TURN_90_TIME;
    }
  }
  
  addToHistory(turnDirection);
  lastTurnDirection = turnDirection;
}

// ==================== INTELLIGENT AUTO NAVIGATION ====================

void intelligentAutoNavigate() {
  if (emergencyStop) return;
  
  unsigned long currentTime = millis();
  
  switch (autoState) {
    case AUTO_MOVING_FORWARD:
      if (currentMotorState != STATE_FORWARD) {
        goForward();
        lastForwardStartTime = currentTime;
        Serial.println("⬆️  Moving forward...");
      }
      
      if (currentDistance != DIST_INVALID && currentDistance < SAFE_DISTANCE) {
        goStop();
        Serial.print("⚠️  Obstacle at ");
        Serial.print(currentDistance);
        Serial.println(" cm");
        autoState = AUTO_OBSTACLE_DETECTED;
        autoStateStartTime = currentTime;
      }
      break;
      
    case AUTO_OBSTACLE_DETECTED:
      goStop();
      if (timeDiff(currentTime, autoStateStartTime) >= 200) {
        autoState = AUTO_SCANNING;
        autoStateStartTime = currentTime;
      }
      break;
      
    case AUTO_SCANNING:
      performIntelligentScan();
      autoState = AUTO_DECIDING;
      autoStateStartTime = currentTime;
      break;
      
    case AUTO_DECIDING:
      makeIntelligentDecision();
      
      // Check if we need dead end recovery (set by makeIntelligentDecision)
      if (inDeadEndRecovery) {
        autoState = AUTO_DEAD_END_RECOVERY;
        autoStateStartTime = currentTime;
        Serial.println("🆘 Entering dead end recovery mode");
      }
      else if (needsBackup) {
        autoState = AUTO_BACKING_UP;
        Serial.println("⬅️  Backing up first...");
        autoStateStartTime = currentTime;
      } else if (turnDirection != 0) {
        autoState = AUTO_TURNING;
        Serial.println("🔄 Executing turn...");
        autoStateStartTime = currentTime;
      } else {
        autoState = AUTO_MOVING_FORWARD;
        Serial.println("✅ Path clear - Resuming");
        autoStateStartTime = currentTime;
      }
      break;
      
    case AUTO_BACKING_UP:
      goBack();
      if (timeDiff(currentTime, autoStateStartTime) >= (needsBackup ? BACKUP_LONG : BACKUP_SHORT)) {
        goStop();
        delay(100);
        
        if (turnDirection != 0) {
          autoState = AUTO_TURNING;
        } else {
          autoState = AUTO_MOVING_FORWARD;
        }
        autoStateStartTime = currentTime;
      }
      break;
      
    case AUTO_TURNING:
      if (turnDirection == -1) {
        goLeft();
      } else if (turnDirection == 1) {
        goRight();
      }
      
      if (timeDiff(currentTime, autoStateStartTime) >= turnAmount) {
        goStop();
        delay(100);
        
        long checkDist = getStableDistance(2);
        if (checkDist != DIST_INVALID && checkDist < SAFE_DISTANCE) {
          Serial.println("⚠️  Path still blocked - Rescanning");
          autoState = AUTO_SCANNING;
          consecutiveStuckCount++;
        } else {
          Serial.println("✅ Turn complete - Moving forward");
          autoState = AUTO_MOVING_FORWARD;
          lastDirectionChangeTime = currentTime;
        }
        autoStateStartTime = currentTime;
      }
      break;
      
    case AUTO_DEAD_END_RECOVERY: {
      // Wrap in braces to create a new scope for variable declarations
      Serial.println("🆘 DEAD END RECOVERY - Executing escape maneuver");
      
      // Aggressive backup
      goBack();
      delay(BACKUP_LONG * 1.5);
      goStop();
      delay(200);
      
      // 180 degree turn (choose direction with more recent success or random)
      int escapeDirection = (lastTurnDirection != 0) ? -lastTurnDirection : (random(0, 2) ? 1 : -1);
      
      Serial.print("   Executing 180° turn ");
      Serial.println(escapeDirection == 1 ? "RIGHT" : "LEFT");
      
      if (escapeDirection == 1) {
        goRight();
      } else {
        goLeft();
      }
      delay(TURN_90_TIME * 2.2);  // Slightly more than 180 to ensure full turn
      goStop();
      delay(200);
      
      // Verify we have space now
      long escapeCheckDist = getStableDistance(3);
      if (escapeCheckDist == DIST_INVALID || escapeCheckDist > SAFE_DISTANCE) {
        Serial.println("✅ Dead end escaped successfully!");
        inDeadEndRecovery = false;
        consecutiveStuckCount = 0;
        failedTurnAttempts = 0;
        deadEndStartTime = 0;
        autoState = AUTO_MOVING_FORWARD;
      } else {
        Serial.println("⚠️  Still blocked after 180° - Trying alternate escape");
        // Try the opposite direction
        if (escapeDirection == 1) {
          goLeft();
        } else {
          goRight();
        }
        delay(TURN_90_TIME * 2.2);
        goStop();
        delay(200);
        
        inDeadEndRecovery = false;
        autoState = AUTO_SCANNING;
      }
      
      autoStateStartTime = currentTime;
      break;
    }
      
    case AUTO_STUCK_RECOVERY: {
      // Wrap in braces to create a new scope
      Serial.println("🆘 Stuck recovery mode");
      goBack();
      delay(BACKUP_LONG);
      goStop();
      delay(100);
      
      int recoveryTurn = random(0, 2) ? 1 : -1;
      if (recoveryTurn == -1) {
        goLeft();
      } else {
        goRight();
      }
      delay(TURN_90_TIME * 2);
      goStop();
      
      consecutiveStuckCount = 0;
      autoState = AUTO_MOVING_FORWARD;
      autoStateStartTime = currentTime;
      break;
    }
  }
}

// ==================== WEB SERVER ====================

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rover Control</title>
  <style>
    body { font-family: Arial; text-align: center; margin: 20px; background: #1a1a1a; color: #fff; }
    h1 { color: #4CAF50; }
    .status { background: #333; padding: 15px; border-radius: 10px; margin: 20px 0; }
    .control-grid { display: grid; grid-template-columns: repeat(3, 100px); gap: 10px; justify-content: center; margin: 20px 0; }
    button { padding: 20px; font-size: 18px; border: none; border-radius: 10px; cursor: pointer; background: #4CAF50; color: white; transition: 0.3s; }
    button:active { background: #45a049; transform: scale(0.95); }
    .mode-btn { background: #2196F3; width: 150px; margin: 5px; }
    .stop-btn { background: #f44336; grid-column: 2; }
    .empty { background: transparent; cursor: default; }
    .speed-control { margin: 20px 0; }
    .speed-control input { width: 200px; }
  </style>
</head>
<body>
  <h1>🤖 Intelligent Rover Control</h1>
  
  <div class="status">
    <h3>Status</h3>
    <p id="status">Mode: MANUAL | Motor: STOPPED</p>
  </div>
  
  <h3>Mode Control</h3>
  <div>
    <button class="mode-btn" onclick="sendCmd('M')">MANUAL</button>
    <button class="mode-btn" onclick="sendCmd('A')">AUTO (AI)</button>
  </div>
  
  <h3>Manual Control</h3>
  <div class="control-grid">
    <div class="empty"></div>
    <button onclick="sendCmd('F')">▲<br>FWD</button>
    <div class="empty"></div>
    
    <button onclick="sendCmd('L')">◄<br>LEFT</button>
    <button class="stop-btn" onclick="sendCmd('S')">■<br>STOP</button>
    <button onclick="sendCmd('R')">►<br>RIGHT</button>
    
    <div class="empty"></div>
    <button onclick="sendCmd('B')">▼<br>BACK</button>
    <div class="empty"></div>
  </div>
  
  <button class="stop-btn" onclick="sendCmd('X')" style="width: 200px; font-size: 20px;">🚨 EMERGENCY STOP</button>
  
  <div class="speed-control">
    <h3>Speed Control</h3>
    <input type="range" min="100" max="200" value="160" id="speedSlider" oninput="updateSpeed(this.value)">
    <p>Speed: <span id="speedValue">160</span></p>
  </div>
  
  <script>
    function sendCmd(cmd) {
      fetch('/cmd?c=' + cmd)
        .then(response => response.text())
        .then(data => console.log(data))
        .catch(err => console.error('Command failed:', err));
    }
    
    function updateSpeed(val) {
      document.getElementById('speedValue').textContent = val;
      sendCmd('S' + val);
    }
    
    function updateStatus() {
      fetch('/status')
        .then(response => response.text())
        .then(data => document.getElementById('status').innerHTML = data)
        .catch(err => {
          console.error('Status update failed:', err);
          document.getElementById('status').innerHTML = 'Connection Lost';
        });
    }
    
    setInterval(updateStatus, 1000);
    updateStatus();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleCommand() {
  if (server.hasArg("c")) {
    String cmd = server.arg("c");
    processCommand(cmd.c_str());
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing command");
  }
}

void handleStatus() {
  String status = "Mode: ";
  if (currentMode == MODE_MANUAL) status += "MANUAL";
  else status += "AUTO (AI)";
  
  status += " | Motor: ";
  if (currentMotorState == STATE_FORWARD) status += "FWD";
  else if (currentMotorState == STATE_BACK) status += "BACK";
  else if (currentMotorState == STATE_LEFT) status += "LEFT";
  else if (currentMotorState == STATE_RIGHT) status += "RIGHT";
  else status += "STOP";
  
  status += " | Dist: ";
  if (currentDistance == DIST_INVALID) status += "N/A";
  else status += String(currentDistance) + "cm";
  
  if (currentMode == MODE_AUTO) {
    status += " | State: ";
    switch(autoState) {
      case AUTO_MOVING_FORWARD: status += "Moving"; break;
      case AUTO_OBSTACLE_DETECTED: status += "Obstacle"; break;
      case AUTO_SCANNING: status += "Scanning"; break;
      case AUTO_DECIDING: status += "Thinking"; break;
      case AUTO_TURNING: status += "Turning"; break;
      case AUTO_BACKING_UP: status += "Backing"; break;
      case AUTO_STUCK_RECOVERY: status += "Recovery"; break;
      case AUTO_DEAD_END_RECOVERY: status += "<span style='color:#ff9800'>DEAD END!</span>"; break;
    }
  }
  
  if (emergencyStop) status += " | <span style='color:#f44336'>E-STOP</span>";
  
  server.send(200, "text/plain", status);
}

void setupWiFi() {
  Serial.println("Setting up WiFi AP...");
  
  WiFi.mode(WIFI_AP);
  delay(100);
  
  if (!WiFi.softAP(ap_ssid, ap_password)) {
    Serial.println("❌ WiFi AP Failed!");
    return;
  }
  
  delay(500);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("✅ AP IP: ");
  Serial.println(IP);
  
  // Start UDP server
  udp.begin(udpPort);
  Serial.print("✅ UDP Server started on port: ");
  Serial.println(udpPort);
  
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/status", handleStatus);
  
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not Found");
  });
  
  server.begin();
  Serial.println("✅ WiFi Ready");
}

// ==================== COMMANDS ====================

void processCommand(const char* cmd) {
  if (strlen(cmd) == 0) return;
  
  String cmdStr = String(cmd);
  cmdStr.trim();
  cmdStr.toUpperCase();
  
  if (cmdStr == "X" || cmdStr == "STOP") {
    emergencyStop = true;
    goStop();
    autoState = AUTO_MOVING_FORWARD;
    Serial.println("🛑 EMERGENCY STOP");
    return;
  }
  
  if (cmdStr == "M" || cmdStr == "A") {
    emergencyStop = false;
    goStop();
    autoState = AUTO_MOVING_FORWARD;
    consecutiveStuckCount = 0;
    inDeadEndRecovery = false;
    deadEndStartTime = 0;
    failedTurnAttempts = 0;
  }
  
  if (cmdStr == "M") {
    currentMode = MODE_MANUAL;
    Serial.println("🎮 MANUAL mode");
    return;
  }
  else if (cmdStr == "A") {
    currentMode = MODE_AUTO;
    autoState = AUTO_MOVING_FORWARD;
    updateTimingConstants();  // Recalculate timings when entering auto mode
    Serial.println("🤖 INTELLIGENT AUTO mode activated");
    return;
  }

  if (emergencyStop) {
    Serial.println("⚠️  COMMAND BLOCKED - Emergency Stop");
    return;
  }

  if (cmdStr == "S") {
    if (currentMode == MODE_MANUAL) {
      goStop();
    }
    return;
  }

  if (cmdStr.startsWith("S") && cmdStr.length() > 1) {
    int newSpeed = cmdStr.substring(1).toInt();
    if (newSpeed >= minSpeed && newSpeed <= maxSpeed) {
      defaultSpeed = newSpeed;
      int proposedTurnSpeed = newSpeed - 10;
      turnSpeed = (proposedTurnSpeed < minSpeed) ? minSpeed : proposedTurnSpeed;
      
      // Update timing constants when speed changes
      if (currentMode == MODE_AUTO) {
        updateTimingConstants();
      }
      
      Serial.print("⚡ Speed: ");
      Serial.println(newSpeed);
    }
    return;
  }

  if (currentMode == MODE_MANUAL) {
    if (cmdStr == "F") {
      goForward();
    }
    else if (cmdStr == "B") {
      goBack();
    }
    else if (cmdStr == "L") {
      goLeft();
    }
    else if (cmdStr == "R") {
      goRight();
    }
  }
}

void handleStream(Stream &stream, CommandBuffer &buf) {
  while (stream.available()) {
    char c = stream.read();
    
    if (c == '\r') continue;
    
    if (buf.index >= RX_BUFFER_SIZE) {
      buf.index = 0;
    }
    
    if (c == '\n') {
      buf.buffer[buf.index] = '\0';
      if (buf.index > 0) {
        processCommand(buf.buffer);
      }
      buf.index = 0;
    } else {
      if (buf.index < RX_BUFFER_SIZE) {
        buf.buffer[buf.index++] = c;
        
        if (buf.index == 1 && strchr(SINGLE_CHAR_CMDS, toupper(c)) != NULL) {
          buf.buffer[1] = '\0';
          processCommand(buf.buffer);
          buf.index = 0;
        }
      }
    }
  }
}

// Handle UDP packets
void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 254);
    if (len > 0) {
      incomingPacket[len] = '\0';
      
      // Ignore PING messages
      if (strcmp(incomingPacket, "PING") == 0) {
        return;
      }
      
      Serial.print("📩 UDP Received: ");
      Serial.println(incomingPacket);
      
      processCommand(incomingPacket);
    }
  }
}

// ==================== SAFETY MONITOR ====================

void safetyMonitor() {
  if (currentMode != MODE_MANUAL) return;
  if (emergencyStop) return;
  
  if (currentMotorState == STATE_FORWARD) {
    long dist = currentDistance;
    
    if (dist != DIST_INVALID && dist < SAFE_DISTANCE) {
      goStop();
      Serial.print("⚠️  OBSTACLE @ ");
      Serial.print(dist);
      Serial.println(" cm - STOPPED");
    }
  }
}

// ==================== MAIN SETUP & LOOP ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║   AI ROVER - DEAD END DETECTION       ║");
  Serial.println("║          v2.2 Enhanced                 ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  setupMotors();
  setupSensor();
  setupWiFi();
  
  // Initialize variables
  currentDistance = DIST_INVALID;
  lastSensorUpdate = 0;
  autoState = AUTO_MOVING_FORWARD;
  autoStateStartTime = 0;
  emergencyStop = false;
  
  turnDirection = 0;
  turnAmount = 0;
  needsBackup = false;
  consecutiveStuckCount = 0;
  lastDirectionChangeTime = 0;
  lastForwardStartTime = 0;
  lastTurnDirection = 0;
  
  // Dead end detection
  deadEndStartTime = 0;
  failedTurnAttempts = 0;
  inDeadEndRecovery = false;
  
  for (int i = 0; i < EXPLORATION_HISTORY; i++) {
    directionHistory[i] = 0;
  }
  historyIndex = 0;
  
  // Initialize timing constants
  updateTimingConstants();
  
  // Seed random number generator
  randomSeed(analogRead(0));
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║  ✅ SYSTEM READY - AWAITING COMMANDS  ║");
  Serial.println("║                                        ║");
  Serial.println("║  M = Manual Mode                       ║");
  Serial.println("║  A = Intelligent Auto Mode             ║");
  Serial.println("║  X = Emergency Stop                    ║");
  Serial.println("║  UDP Port: 1234                        ║");
  Serial.println("║                                        ║");
  Serial.println("║  Features:                             ║");
  Serial.println("║  • Speed-adaptive turning              ║");
  Serial.println("║  • Dead-end detection & recovery       ║");
  Serial.println("║  • UDP gesture control support         ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

void loop() {
  // Handle UDP commands (for gesture control)
  handleUDP();
  
  // Handle serial commands
  if (Serial.available()) {
    handleStream(Serial, serialBuffer);
  }
  
  // Handle web server
  server.handleClient();
  
  // Always update sensor
  updateSensor();
  
  // Safety monitor for manual mode
  safetyMonitor();
  
  // Intelligent auto navigation
  if (currentMode == MODE_AUTO && !emergencyStop) {
    intelligentAutoNavigate();
  }
  
  // Handle mode changes
  if (currentMode != previousMode) {
    Serial.print("🔄 Mode: ");
    Serial.println(currentMode == MODE_MANUAL ? "MANUAL" : "INTELLIGENT AUTO");
    
    goStop();
    autoState = AUTO_MOVING_FORWARD;
    consecutiveStuckCount = 0;
    inDeadEndRecovery = false;
    deadEndStartTime = 0;
    failedTurnAttempts = 0;
    
    if (currentMode == MODE_AUTO) {
      updateTimingConstants();
    }
    
    previousMode = currentMode;
  }
  
  // Periodic status updates
  if (timeDiff(millis(), lastStatusPrint) > STATUS_PRINT_INTERVAL) {
    lastStatusPrint = millis();
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.print("║ Mode: ");
    Serial.print(currentMode == MODE_MANUAL ? "MANUAL  " : "AUTO    ");
    Serial.print(" | Motor: ");
    
    switch(currentMotorState) {
      case STATE_FORWARD: Serial.print("FWD    "); break;
      case STATE_BACK: Serial.print("BACK   "); break;
      case STATE_LEFT: Serial.print("LEFT   "); break;
      case STATE_RIGHT: Serial.print("RIGHT  "); break;
      default: Serial.print("STOP   "); break;
    }
    Serial.println("║");
    
    Serial.print("║ Distance: ");
    if (currentDistance == DIST_INVALID) {
      Serial.print("N/A    ");
    } else {
      Serial.print(currentDistance);
      Serial.print(" cm   ");
      if (currentDistance < 10) Serial.print(" ");
      if (currentDistance < 100) Serial.print(" ");
    }
    
    if (currentMode == MODE_AUTO) {
      Serial.print(" | AI State: ");
      switch(autoState) {
        case AUTO_MOVING_FORWARD: Serial.print("Moving   "); break;
        case AUTO_OBSTACLE_DETECTED: Serial.print("Obstacle "); break;
        case AUTO_SCANNING: Serial.print("Scanning "); break;
        case AUTO_DECIDING: Serial.print("Thinking "); break;
        case AUTO_TURNING: Serial.print("Turning  "); break;
        case AUTO_BACKING_UP: Serial.print("Backing  "); break;
        case AUTO_STUCK_RECOVERY: Serial.print("Recovery "); break;
        case AUTO_DEAD_END_RECOVERY: Serial.print("DEAD END!"); break;
      }
    } else {
      Serial.print("                       ");
    }
    Serial.println("║");
    
    if (emergencyStop) {
      Serial.println("║              ⚠️  E-STOP ACTIVE ⚠️              ║");
    }
    
    if (inDeadEndRecovery) {
      Serial.println("║           🚧 DEAD END RECOVERY MODE 🚧         ║");
    }
    
    Serial.println("╚════════════════════════════════════════╝\n");
  }
  
  delay(10);
}