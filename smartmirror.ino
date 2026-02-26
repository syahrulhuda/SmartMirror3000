/**
 * Smart Mirror Control System
 *
 * This program manages the brightness of a smart mirror system based on
 * distance and proximity sensors. It uses an ultrasonic sensor to detect user
 * presence and transitions the system between IDLE, ACTIVE, and ALWAYS_ON
 * states. In the ACTIVE state, two proximity sensors can detect hand swipe
 * gestures (left-to-right or right-to-left) to adjust the brightness level. A
 * third proximity sensor or button acts as a toggle for the ALWAYS_ON mode. It
 * features debouncing for sensors and an exponential moving average (EMA)
 * filter for distance measurement smoothing.
 */

#define TRIGGER_PIN 18
#define ECHO_PIN 19
#define PWM_PIN 15
#define PROXIMITY_1_PIN 32
#define PROXIMITY_2_PIN 33
#define PROXIMITY_3_PIN 34

const int THRESHOLD_DISTANCE = 80;
const int DEFAULT_BRIGHTNESS = 127;
const int BRIGHTNESS_STEP = 50;
const int PROXIMITY_SEQUENCE_TIMEOUT = 2000;
const int DETECTION_DELAY = 2000;
const int IDLE_DELAY = 2000;

const unsigned long DEBOUNCE_DELAY = 50;
const int REQUIRED_SAMPLES = 3;
const float DISTANCE_ALPHA = 0.2;
const unsigned long BUTTON_COOLDOWN = 500;
const unsigned long BRIGHTNESS_UPDATE_INTERVAL = 50;

enum SystemState { STATE_IDLE, STATE_DETECTING, STATE_ACTIVE, STATE_ALWAYS_ON };

enum ProximityState { WAIT_FIRST_SENSOR, WAIT_SECOND_SENSOR };

struct SystemContext {
  int targetBrightness = 0;
  int currentBrightness = 0;
  float smoothedDistance = -1;

  SystemState systemState = STATE_IDLE;
  ProximityState proximityState = WAIT_FIRST_SENSOR;

  unsigned long detectionStartTime = 0;
  unsigned long firstSensorDetectionTime = 0;
  unsigned long lastUpdateTime = 0;
  unsigned long lastDetectionTime = 0;
  unsigned long lastProximity3Time = 0;

  bool lastProximity1State = false;
  bool lastProximity2State = false;
  bool lastProximity3State = false;
  bool lastProximity3Value = false;

  int firstActiveSensor = 0;
  bool alwaysOnMode = false;
} sys;

bool hasTimeElapsed(unsigned long current, unsigned long previous,
                    unsigned long interval) {
  return (current - previous) >= interval;
}

bool debounceRead(int pin, bool &lastState) {
  bool stableState = lastState;
  int stableCount = 0;
  bool initialReading = digitalRead(pin);

  if (initialReading != lastState) {
    for (int i = 0; i < REQUIRED_SAMPLES; i++) {
      delay(DEBOUNCE_DELAY / REQUIRED_SAMPLES);
      if (digitalRead(pin) == initialReading) {
        stableCount++;
      }
    }

    if (stableCount >= REQUIRED_SAMPLES - 1) {
      stableState = initialReading;
    }
  }

  lastState = stableState;
  return stableState;
}

int measureDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return sys.smoothedDistance < 0 ? -1 : round(sys.smoothedDistance);
  }

  float distance = duration * 0.034 / 2;

  if (distance < 2 || distance > 400) {
    return sys.smoothedDistance < 0 ? -1 : round(sys.smoothedDistance);
  }

  if (sys.smoothedDistance < 0) {
    sys.smoothedDistance = distance;
  } else {
    sys.smoothedDistance = (DISTANCE_ALPHA * distance) +
                           ((1 - DISTANCE_ALPHA) * sys.smoothedDistance);
  }

  return round(sys.smoothedDistance);
}

void transitToState(SystemState newState) {
  if (sys.systemState == newState)
    return;

  switch (sys.systemState) {
  case STATE_ACTIVE:
  case STATE_ALWAYS_ON:
    if (newState == STATE_IDLE) {
      sys.targetBrightness = 0;
      sys.alwaysOnMode = false;
    }
    break;
  default:
    break;
  }

  sys.systemState = newState;
  switch (newState) {
  case STATE_ACTIVE:
    sys.lastDetectionTime = millis();
    setBrightness(DEFAULT_BRIGHTNESS);
    Serial.println(F("Sistem Aktif"));
    break;
  case STATE_ALWAYS_ON:
    sys.alwaysOnMode = true;
    Serial.println(F("Masuk Always On Mode"));
    break;
  case STATE_IDLE:
    sys.currentBrightness = 0;
    analogWrite(PWM_PIN, 0);
    Serial.println(F("Sistem Non-Aktif"));
    break;
  default:
    break;
  }
}

void updateBrightness() {
  unsigned long currentTime = millis();

  if (hasTimeElapsed(currentTime, sys.lastUpdateTime,
                     BRIGHTNESS_UPDATE_INTERVAL)) {
    if (sys.currentBrightness != sys.targetBrightness) {
      int diff = sys.targetBrightness - sys.currentBrightness;
      int step = max(1, abs(diff) / 8);

      if (diff > 0) {
        sys.currentBrightness += step;
      } else if (diff < 0) {
        sys.currentBrightness -= step;
      }

      sys.currentBrightness = constrain(sys.currentBrightness, 0, 255);
      analogWrite(PWM_PIN, sys.currentBrightness);
    }
    sys.lastUpdateTime = currentTime;
  }
}

void setBrightness(int newBrightness) {
  sys.targetBrightness = constrain(newBrightness, 0, 255);
  Serial.printf(F("Kecerahan: %d (%.1f%%)\n"), sys.targetBrightness,
                (sys.targetBrightness * 100.0) / 255);
}

void processProximitySequence(bool proximity1, bool proximity2) {
  unsigned long currentTime = millis();

  switch (sys.proximityState) {
  case WAIT_FIRST_SENSOR:
    if (proximity1 || proximity2) {
      sys.firstSensorDetectionTime = currentTime;
      sys.proximityState = WAIT_SECOND_SENSOR;
      sys.firstActiveSensor = proximity1 ? PROXIMITY_1_PIN : PROXIMITY_2_PIN;
      Serial.println(F("Sensor Pertama Terdeteksi"));
    }
    break;

  case WAIT_SECOND_SENSOR:
    if (hasTimeElapsed(currentTime, sys.firstSensorDetectionTime,
                       PROXIMITY_SEQUENCE_TIMEOUT)) {
      sys.proximityState = WAIT_FIRST_SENSOR;
      Serial.println(F("Proximity Timeout"));
      return;
    }

    if (sys.firstActiveSensor == PROXIMITY_1_PIN && !proximity1 && proximity2) {
      setBrightness(sys.targetBrightness - BRIGHTNESS_STEP);
      sys.proximityState = WAIT_FIRST_SENSOR;
      Serial.println(F("Kiri ke Kanan - Turunkan Kecerahan"));
    } else if (sys.firstActiveSensor == PROXIMITY_2_PIN && !proximity2 &&
               proximity1) {
      setBrightness(sys.targetBrightness + BRIGHTNESS_STEP);
      sys.proximityState = WAIT_FIRST_SENSOR;
      Serial.println(F("Kanan ke Kiri - Naikkan Kecerahan"));
    }
    break;
  }
}

void handleAlwaysOnMode(unsigned long currentTime) {
  bool proximity3 = debounceRead(PROXIMITY_3_PIN, sys.lastProximity3State);

  if (proximity3 && !sys.lastProximity3Value &&
      hasTimeElapsed(currentTime, sys.lastProximity3Time, BUTTON_COOLDOWN)) {
    transitToState(STATE_IDLE);
    sys.lastProximity3Time = currentTime;
  }
  sys.lastProximity3Value = proximity3;
}

void handleActiveMode(unsigned long currentTime) {
  bool proximity3 = debounceRead(PROXIMITY_3_PIN, sys.lastProximity3State);

  if (proximity3 && !sys.lastProximity3Value &&
      hasTimeElapsed(currentTime, sys.lastProximity3Time, BUTTON_COOLDOWN)) {
    transitToState(STATE_ALWAYS_ON);
    sys.lastProximity3Time = currentTime;
    sys.lastProximity3Value = proximity3;
    return;
  }
  sys.lastProximity3Value = proximity3;

  int distance = measureDistance();
  bool proximity1 = debounceRead(PROXIMITY_1_PIN, sys.lastProximity1State);
  bool proximity2 = debounceRead(PROXIMITY_2_PIN, sys.lastProximity2State);

  if (distance > 0 && distance < THRESHOLD_DISTANCE) {
    processProximitySequence(proximity1, proximity2);
    sys.lastDetectionTime = currentTime;
  } else if (hasTimeElapsed(currentTime, sys.lastDetectionTime, IDLE_DELAY)) {
    transitToState(STATE_IDLE);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PROXIMITY_1_PIN, INPUT);
  pinMode(PROXIMITY_2_PIN, INPUT);
  pinMode(PROXIMITY_3_PIN, INPUT);

  digitalWrite(TRIGGER_PIN, LOW);
  analogWrite(PWM_PIN, 0);
  sys.smoothedDistance = -1;

  Serial.println(F("Sistem Started"));
}

void loop() {
  unsigned long currentTime = millis();
  int distance;

  updateBrightness();

  switch (sys.systemState) {
  case STATE_IDLE:
    distance = measureDistance();

    if (distance > 0 && distance < THRESHOLD_DISTANCE) {
      if (sys.detectionStartTime == 0) {
        sys.detectionStartTime = currentTime;
      }

      if (hasTimeElapsed(currentTime, sys.detectionStartTime,
                         DETECTION_DELAY)) {
        transitToState(STATE_ACTIVE);
      }
    } else {
      sys.detectionStartTime = 0;
    }
    break;

  case STATE_ACTIVE:
    handleActiveMode(currentTime);
    break;

  case STATE_ALWAYS_ON:
    handleAlwaysOnMode(currentTime);
    break;
  }
}