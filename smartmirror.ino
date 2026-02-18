#define TRIGGER_PIN 18
#define ECHO_PIN 19
#define PWM_PIN 15
#define PROXIMITY_1_PIN 32
#define PROXIMITY_2_PIN 33
#define PROXIMITY_3_PIN 34  // Push-Lock Button Pin

// Konfigurasi sistem
const int THRESHOLD_DISTANCE = 80;        // Jarak deteksi maks (cm)
const int DEFAULT_BRIGHTNESS = 127;        // Kecerahan default 50%
const int BRIGHTNESS_STEP = 50;            // Perubahan kecerahan per step
const int PROXIMITY_SEQUENCE_TIMEOUT = 2000; // Timeout sequence proximity (ms)
const int DETECTION_DELAY = 2000;           // Waktu deteksi untuk aktivasi (2 detik)
const int IDLE_DELAY = 2000;                // Waktu tunggu sebelum non-aktif (2 detik)

// Konstanta untuk debouncing
const unsigned long DEBOUNCE_DELAY = 50;    // Delay antar pembacaan untuk debouncing
const int REQUIRED_SAMPLES = 3;             // Jumlah sampel yang dibutuhkan untuk konfirmasi
const float DISTANCE_ALPHA = 0.2;           // Konstanta filter untuk pengukuran jarak
const unsigned long BUTTON_COOLDOWN = 500;  // Cooldown untuk tombol (ms)
const unsigned long BRIGHTNESS_UPDATE_INTERVAL = 50; // Interval update kecerahan (ms)

// Enum untuk state sistem
enum SystemState {
    STATE_IDLE,
    STATE_DETECTING,
    STATE_ACTIVE,
    STATE_ALWAYS_ON
};

enum ProximityState {
    WAIT_FIRST_SENSOR,
    WAIT_SECOND_SENSOR
};

// Struktur data global
struct SystemContext {
    int targetBrightness = 0;
    int currentBrightness = 0;
    float smoothedDistance = -1;  // Untuk EMA filter

    SystemState systemState = STATE_IDLE;
    ProximityState proximityState = WAIT_FIRST_SENSOR;

    unsigned long detectionStartTime = 0;
    unsigned long firstSensorDetectionTime = 0;
    unsigned long lastUpdateTime = 0;
    unsigned long lastDetectionTime = 0;
    unsigned long lastProximity3Time = 0;
    
    // Debouncing status untuk proximity sensors
    bool lastProximity1State = false;
    bool lastProximity2State = false;
    bool lastProximity3State = false;
    bool lastProximity3Value = false;  // Untuk edge detection
    
    int firstActiveSensor = 0;
    bool alwaysOnMode = false;
} sys;

// Fungsi untuk handle millis() overflow
bool hasTimeElapsed(unsigned long current, unsigned long previous, unsigned long interval) {
    return (current - previous) >= interval;
}

// Fungsi debouncing yang diperbaiki
bool debounceRead(int pin, bool &lastState) {
    bool stableState = lastState;
    int stableCount = 0;
    bool initialReading = digitalRead(pin);
    
    // Hanya lakukan multiple sampling jika ada perubahan state
    if (initialReading != lastState) {
        for(int i = 0; i < REQUIRED_SAMPLES; i++) {
            delay(DEBOUNCE_DELAY / REQUIRED_SAMPLES);
            if(digitalRead(pin) == initialReading) {
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

// Fungsi pengukuran jarak dengan error handling yang lebih baik
int measureDistance() {
    // Generate trigger pulse
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // Read echo pulse
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
    
    // Handle timeout atau error
    if (duration == 0) {
        return sys.smoothedDistance < 0 ? -1 : round(sys.smoothedDistance);
    }
    
    // Calculate distance
    float distance = duration * 0.034 / 2;
    
    // Filter bad readings
    if (distance < 2 || distance > 400) {
        return sys.smoothedDistance < 0 ? -1 : round(sys.smoothedDistance);
    }
    
    // Apply EMA filter
    if(sys.smoothedDistance < 0) {
        sys.smoothedDistance = distance;
    } else {
        sys.smoothedDistance = (DISTANCE_ALPHA * distance) + 
                             ((1-DISTANCE_ALPHA) * sys.smoothedDistance);
    }
    
    return round(sys.smoothedDistance);
}

// Fungsi transisi state dengan cleanup yang diperbaiki
void transitToState(SystemState newState) {
    // Prevent unnecessary state transitions
    if (sys.systemState == newState) return;
    
    // Cleanup state lama
    switch(sys.systemState) {
        case STATE_ACTIVE:
        case STATE_ALWAYS_ON:
            if(newState == STATE_IDLE) {
                sys.targetBrightness = 0;
                sys.alwaysOnMode = false;
            }
            break;
        default:
            break;
    }
    
    // Setup state baru
    sys.systemState = newState;
    switch(newState) {
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

// Fungsi update kecerahan dengan smoothing yang diperbaiki
void updateBrightness() {
    unsigned long currentTime = millis();
    
    if (hasTimeElapsed(currentTime, sys.lastUpdateTime, BRIGHTNESS_UPDATE_INTERVAL)) {
        if (sys.currentBrightness != sys.targetBrightness) {
            // Smooth transition logic
            int diff = sys.targetBrightness - sys.currentBrightness;
            int step = max(1, abs(diff) / 8);  // Minimal step 1, max 1/8 of difference
            
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

// Fungsi atur kecerahan dengan validasi
void setBrightness(int newBrightness) {
    sys.targetBrightness = constrain(newBrightness, 0, 255);
    Serial.printf(F("Kecerahan: %d (%.1f%%)\n"), 
                 sys.targetBrightness, 
                 (sys.targetBrightness * 100.0) / 255);
}

// Proses sequence proximity dengan timeout handling yang diperbaiki
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
            // Improved timeout check
            if (hasTimeElapsed(currentTime, sys.firstSensorDetectionTime, PROXIMITY_SEQUENCE_TIMEOUT)) {
                sys.proximityState = WAIT_FIRST_SENSOR;
                Serial.println(F("Proximity Timeout"));
                return;
            }

            // Process sequence
            if (sys.firstActiveSensor == PROXIMITY_1_PIN && !proximity1 && proximity2) {
                setBrightness(sys.targetBrightness - BRIGHTNESS_STEP);
                sys.proximityState = WAIT_FIRST_SENSOR;
                Serial.println(F("Kiri ke Kanan - Turunkan Kecerahan"));
            } 
            else if (sys.firstActiveSensor == PROXIMITY_2_PIN && !proximity2 && proximity1) {
                setBrightness(sys.targetBrightness + BRIGHTNESS_STEP);
                sys.proximityState = WAIT_FIRST_SENSOR;
                Serial.println(F("Kanan ke Kiri - Naikkan Kecerahan"));
            }
            break;
    }
}

// Handler untuk mode Always On dengan edge detection
void handleAlwaysOnMode(unsigned long currentTime) {
    bool proximity3 = debounceRead(PROXIMITY_3_PIN, sys.lastProximity3State);
    
    // Edge detection untuk tombol
    if (proximity3 && !sys.lastProximity3Value && 
        hasTimeElapsed(currentTime, sys.lastProximity3Time, BUTTON_COOLDOWN)) {
        transitToState(STATE_IDLE);
        sys.lastProximity3Time = currentTime;
    }
    sys.lastProximity3Value = proximity3;
}

// Handler untuk mode Active dengan edge detection
void handleActiveMode(unsigned long currentTime) {
    bool proximity3 = debounceRead(PROXIMITY_3_PIN, sys.lastProximity3State);
    
    // Edge detection untuk tombol
    if (proximity3 && !sys.lastProximity3Value && 
        hasTimeElapsed(currentTime, sys.lastProximity3Time, BUTTON_COOLDOWN)) {
        transitToState(STATE_ALWAYS_ON);
        sys.lastProximity3Time = currentTime;
        sys.lastProximity3Value = proximity3;
        return;
    }
    sys.lastProximity3Value = proximity3;

    // Process distance and proximity sensors
    int distance = measureDistance();
    bool proximity1 = debounceRead(PROXIMITY_1_PIN, sys.lastProximity1State);
    bool proximity2 = debounceRead(PROXIMITY_2_PIN, sys.lastProximity2State);

    if (distance > 0 && distance < THRESHOLD_DISTANCE) {
        processProximitySequence(proximity1, proximity2);
        sys.lastDetectionTime = currentTime;
    } 
    else if (hasTimeElapsed(currentTime, sys.lastDetectionTime, IDLE_DELAY)) {
        transitToState(STATE_IDLE);
    }
}

void setup() {
    Serial.begin(115200);

    // Pin setup
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(PROXIMITY_1_PIN, INPUT);
    pinMode(PROXIMITY_2_PIN, INPUT);
    pinMode(PROXIMITY_3_PIN, INPUT);

    // Initial state
    digitalWrite(TRIGGER_PIN, LOW);
    analogWrite(PWM_PIN, 0);
    sys.smoothedDistance = -1;
    
    Serial.println(F("Sistem Started"));
}

void loop() {
    unsigned long currentTime = millis();
    int distance;  // Deklarasi di luar switch
    
    // Update kecerahan selalu aktif
    updateBrightness();

    // State machine utama
    switch (sys.systemState) {
        case STATE_IDLE:
            distance = measureDistance();
            
            if (distance > 0 && distance < THRESHOLD_DISTANCE) {
                if (sys.detectionStartTime == 0) {
                    sys.detectionStartTime = currentTime;
                }

                if (hasTimeElapsed(currentTime, sys.detectionStartTime, DETECTION_DELAY)) {
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