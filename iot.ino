// ------------------- SETTINGS -------------------
#include "RAK12027_D7S.h" // RAK D7S library
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

// WiFi Credentials
char ssid[] = "WIFI_SSID";
char pass[] = "WIFI_PASSWORD";

// API Configuration
const char* serverURL = "BACKEND_URL";
const char* emailURL = "EMAIL_URL";
const char* earthquakeURL = "EARTHQUAKE_URL";
const char* authToken = "AUTH_TOKEN";
const char* tokenType = "TOKEN_TYPE";

// ------------------- D7S SENSOR -----------------------
RAK_D7S D7S;

// ------------------- VARIABLES ------------------------
unsigned long lastAverageTime = 0;
const unsigned long averageInterval = 5 * 60 * 1000UL; // 5 minutes

// Define buzzer pin and variables
const int BUZZER_PIN = WB_IO5;
unsigned long lastBuzzerTest = 0;
const unsigned long buzzerTestInterval = 60 * 60 * 1000UL; // Test buzzer every hour

// Dynamic data collection arrays
float readings[100]; // Store last 100 readings
int readingIndex = 0;
int totalReadings = 0;

float batteryLevel = 100.0;
int signalStrength = 0;

bool earthquakeActive = false;
unsigned long earthquakeStart = 0;
const unsigned long earthquakeHoldTime = 5000; // 5 seconds for display purposes only
bool earthquakeTriggeredOnce = false;

// EARTHQUAKE DURATION TRACKING
unsigned long earthquakeDurationMs = 0; // Track duration for API reporting

// EMAIL TRACKING VARIABLES
bool emailSent = false;
float lastEmailMagnitude = 0.0;
unsigned long lastEmailTime = 0;
// const unsigned long emailCooldown = 10 * 60 * 1000UL; // DISABLED: No cooldown between emails

bool wifiConnected = false;
bool sensorReady = false;

// Sensor retry variables
unsigned long lastSensorRetry = 0;
const unsigned long sensorRetryInterval = 60 * 1000UL;

// Sensor validation variables
int consecutiveInvalidReadings = 0;
const int maxInvalidReadings = 5;

// Stuck reading detection
float lastSensorReading = -1;
int stuckReadingCount = 0;
const int maxStuckReadings = 10;
float stuckValue = -1;
bool sensorStuck = false;

// MAGNITUDE CONVERSION FUNCTION
float convertSIToMagnitude(float si) {
    // Based on seismic research correlations
    if (si <= 0.5) {
        return 1.0 + (si * 2.0); // M1.0-2.0
    } else if (si <= 1.5) {
        return 2.0 + ((si - 0.5) * 1.0); // M2.0-3.0
    } else if (si <= 3.0) {
        return 3.0 + ((si - 1.5) / 1.5); // M3.0-4.0
    } else if (si <= 6.0) {
        return 4.0 + ((si - 3.0) / 3.0); // M4.0-5.0
    } else {
        return 5.0 + ((si - 6.0) / 4.0); // M5.0+
    }
}

// ------------------- SEND EARTHQUAKE DATA TO API -----------------
void sendEarthquakeToAPI(float magnitude, unsigned long durationMs) {
    if (!wifiConnected) {
        safePrint("📱 Offline - earthquake data not sent");
        return;
    }

    // Convert milliseconds to seconds for the API
    int durationSeconds = durationMs / 1000;

    // Ensure minimum 1 second duration
    if (durationSeconds < 1) {
        durationSeconds = 1;
    }

    HTTPClient http;
    http.begin(earthquakeURL);
    http.setTimeout(10000);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + authToken);
    http.addHeader("Token-Type", tokenType);

    String payload = "{";
    payload += "\"magnitude\":" + String(magnitude, 1) + ",";
    payload += "\"duration\":" + String(durationSeconds);
    payload += "}";

    safePrint("📤 Sending earthquake data: " + payload);

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
        if (httpResponseCode == 201) {
            safePrint("✅ Earthquake data sent successfully!");
        } else {
            String response = http.getString();
            safePrint("⚠ API Error " + String(httpResponseCode) + ": " + response);
        }
    } else {
        safePrint("❌ HTTP Error: " + String(httpResponseCode));
    }

    http.end();
}

// EMAIL SENDING FUNCTION
void sendEarthquakeEmail(float magnitude) {
    if (!wifiConnected) {
        safePrint("📱 Offline - email not sent");
        return;
    }

    // DISABLED: Check cooldown period
    // if (millis() - lastEmailTime < emailCooldown) {
    //     safePrint("⏰ Email cooldown active - skipping (next email in " +
    //               String((emailCooldown - (millis() - lastEmailTime)) / 1000) + " seconds)");
    //     return;
    // }

    HTTPClient http;
    http.begin(emailURL);
    http.setTimeout(15000); // 15 second timeout for email
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + authToken);
    http.addHeader("Token-Type", tokenType);

    String payload = "{";
    payload += "\"magnitude\":" + String(magnitude, 1);
    payload += "}";

    safePrint("📧 Sending earthquake email: " + payload);

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
        if (httpResponseCode == 200) {
            safePrint("✅ Email sent successfully!");
            lastEmailTime = millis();
            lastEmailMagnitude = magnitude;
            emailSent = true;
        } else {
            String response = http.getString();
            safePrint("⚠ Email Error " + String(httpResponseCode) + ": " + response);
        }
    } else {
        safePrint("❌ Email HTTP Error: " + String(httpResponseCode));
    }

    http.end();
}

// ------------------- SAFE SERIAL PRINT ---------------
void safePrint(String message) {
    if (Serial) {
        Serial.println(message);
        Serial.flush();
    }
}

// ------------------- I2C SCANNER ---------------------
void scanI2C() {
    safePrint("🔍 Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            if (Serial) {
                Serial.print("I2C device found at address 0x");
                if (address < 16) Serial.print("0");
                Serial.println(address, HEX);
                Serial.flush();
            }
            nDevices++;
        }
    }

    if (nDevices == 0) {
        safePrint("❌ No I2C devices found");
    } else {
        safePrint("✅ Found " + String(nDevices) + " I2C device(s)");
    }
}

// ------------------- SETUP ----------------------------
void setup() {
    Serial.begin(115200);
    delay(3000);

    safePrint("\n🔄 === DEVICE STARTUP ===");
    safePrint("Board: ESP32-based device");

    // Initialize I2C
    safePrint("🔧 Initializing I2C...");
    Wire.begin();
    Wire.setClock(100000);
    delay(1000);

    scanI2C();

    // Initialize power control
    safePrint("⚡ Configuring power control...");
    pinMode(WB_IO2, OUTPUT);

    // Initialize buzzer
    safePrint("🔊 Configuring buzzer...");
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off at startup

    // Test buzzer at startup
    safePrint("🔊 Testing buzzer at startup...");
    tone(BUZZER_PIN, 2000, 200);  // Short beep
    delay(200);
    tone(BUZZER_PIN, 2500, 200);  // Second short beep
    safePrint("🔊 Buzzer test complete");

    // Power cycle sensor
    for (int cycle = 0; cycle < 3; cycle++) {
        safePrint("Power cycle " + String(cycle + 1) + "/3");
        digitalWrite(WB_IO2, LOW);
        delay(2000);
        digitalWrite(WB_IO2, HIGH);
        delay(3000);
        scanI2C();
    }

    // Initialize WiFi
    wifiConnected = connectWiFi();

    // Try different I2C speeds
    safePrint("🔧 Trying different I2C configurations...");
    int speeds[] = {100000, 400000, 50000};
    for (int i = 0; i < 3; i++) {
        safePrint("Trying I2C speed: " + String(speeds[i]) + " Hz");
        Wire.setClock(speeds[i]);
        delay(500);

        if (initializeSensor()) {
            sensorReady = true;
            safePrint("✅ Sensor initialized at " + String(speeds[i]) + " Hz");
            break;
        }
    }

    if (!sensorReady) {
        safePrint("❌ All initialization attempts failed");
        safePrint("⚠️ SENSOR REQUIRED - System will only log sensor data when available");
    }

    // Initialize readings array
    for (int i = 0; i < 100; i++) {
        readings[i] = 0.0;
    }

    lastAverageTime = millis();
    lastSensorRetry = millis();
    lastBuzzerTest = millis();
    consecutiveInvalidReadings = 0;
    lastSensorReading = -1;
    stuckReadingCount = 0;
    earthquakeTriggeredOnce = false;
    sensorStuck = false;
    stuckValue = -1;



    safePrint("======================\n");
}

// ------------------- WIFI CONNECTION ------------------
bool connectWiFi() {
    safePrint("Connecting to WiFi: " + String(ssid));

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        delay(500);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        safePrint("✅ WiFi connected: " + WiFi.localIP().toString());
        signalStrength = WiFi.RSSI();
        return true;
    } else {
        safePrint("❌ WiFi failed");
        return false;
    }
}

// ------------------- SENSOR INITIALIZATION ------------
bool initializeSensor() {
    safePrint("🔧 Attempting D7S initialization...");

    Wire.beginTransmission(0x55);
    byte error = Wire.endTransmission();

    if (error != 0) {
        safePrint("❌ No response from D7S at address 0x55, error: " + String(error));
        return false;
    } else {
        safePrint("✅ D7S responds to I2C");
    }

    if (!D7S.begin()) {
        safePrint("❌ D7S.begin() failed");
        return false;
    }

    safePrint("✅ D7S.begin() successful");

    D7S.setAxis(SWITCH_AT_INSTALLATION);
    delay(2000);
    D7S.initialize();
    delay(1000);

    safePrint("Waiting for sensor ready...");
    for (int check = 0; check < 60; check++) {
        if (D7S.isReady()) {
            safePrint("✅ Sensor READY!");
            return true;
        }
        delay(1000);
    }

    safePrint("❌ Sensor TIMEOUT");
    return false;
}

// ------------------- ADD READING TO BUFFER ------------
void addReading(float si) {
    readings[readingIndex] = si;
    readingIndex = (readingIndex + 1) % 100; // Circular buffer
    if (totalReadings < 100) totalReadings++;
}

// ------------------- CALCULATE STATISTICS -------------
void calculateStats(float &average, float &minimum, float &maximum) {
    if (totalReadings == 0) {
        average = minimum = maximum = 0;
        return;
    }

    float sum = 0;
    minimum = readings[0];
    maximum = readings[0];

    for (int i = 0; i < totalReadings; i++) {
        sum += readings[i];
        if (readings[i] < minimum) minimum = readings[i];
        if (readings[i] > maximum) maximum = readings[i];
    }

    average = sum / totalReadings;
}

// ------------------- SEND DATA TO API -----------------
void sendReadingToAPI(float avgSI, float minSI, float maxSI) {
    if (!wifiConnected) {
        safePrint("📱 Offline - data not sent");
        return;
    }

    HTTPClient http;
    http.begin(serverURL);
    http.setTimeout(10000);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + authToken);
    http.addHeader("Token-Type", tokenType);

    String payload = "{";
    payload += "\"siAverage\":" + String(avgSI, 3) + ",";
    payload += "\"siMinimum\":" + String(minSI, 3) + ",";
    payload += "\"siMaximum\":" + String(maxSI, 3) + ",";
    payload += "\"battery\":" + String(batteryLevel, 1) + ",";
    payload += "\"signalStrength\":\"" + String(signalStrength) + "\"";
    payload += "}";

    safePrint("📤 Sending: " + payload);

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
        if (httpResponseCode == 201) {
            safePrint("✅ API Success!");
        } else {
            String response = http.getString();
            safePrint("⚠ API Error " + String(httpResponseCode) + ": " + response);
        }
    } else {
        safePrint("❌ HTTP Error: " + String(httpResponseCode));
    }

    http.end();
}

// ------------------- EARTHQUAKE DETECTION (SENSOR ONLY) ----
void detectEarthquakeForDisplay(float si) {
    // Always process valid readings, removed stuck check
    // The original code had:
    // if (sensorStuck) {
    //     return; // Skip earthquake detection when sensor is stuck
    // }

    // Check for earthquake trigger (for display/logging purposes only)
    if (si >= 0.5 && !earthquakeActive && !earthquakeTriggeredOnce) {
        earthquakeActive = true;
        earthquakeStart = millis();
        earthquakeTriggeredOnce = true;

        // Convert SI to magnitude
        float magnitude = convertSIToMagnitude(si);

        safePrint("🚨 EARTHQUAKE DETECTED! SI: " + String(si, 3) + " | Magnitude: " + String(magnitude, 1));
        safePrint("⏰ Display alert will clear in " + String(earthquakeHoldTime/1000) + " seconds");
        safePrint("📊 Backend will detect earthquake from 5-minute siMaximum data");

        // Sound the buzzer alarm with retries to ensure it works
        safePrint("  ACTIVATING BUZZER ALARM - EARTHQUAKE ALERT!");

        // Play alarm sound just once, no repeat attempts
        soundEarthquakeAlarm();
        // Removed loop that was causing multiple alarms

        // Send email notification (only if not already sent for this earthquake)
        if (magnitude >= 2.0 && !emailSent) { // Send email for M2.5+ earthquakes (changed from 3.0)
            safePrint("📧 Sending email notification for M" + String(magnitude, 1) + " earthquake");
            sendEarthquakeEmail(magnitude);
        } else if (magnitude < 2.5) {
            safePrint("📧 Skipping email for minor earthquake (M" + String(magnitude, 1) + ")");
        } else if (emailSent) {
            safePrint("📧 Email already sent for this earthquake event");
        }
    }

    // Check for earthquake clearing (display only)
    if (earthquakeActive) {
        unsigned long alertDuration = millis() - earthquakeStart;

        // Show countdown every second during alert
        if (alertDuration % 1000 == 0) {
            unsigned long remainingTime = earthquakeHoldTime - alertDuration;
            if (remainingTime > 0) {
                float magnitude = convertSIToMagnitude(si);
                safePrint("🚨 EARTHQUAKE ACTIVE - Clearing display in: " + String(remainingTime/1000) + "s (SI: " + String(si, 3) + " | M" + String(magnitude, 1) + ")");
            }
        }

        // Clear earthquake display after hold time
        if (alertDuration >= earthquakeHoldTime) {
            // Calculate actual earthquake duration (from start to now)
            earthquakeDurationMs = alertDuration;

            // Convert SI to magnitude for final reading
            float magnitude = convertSIToMagnitude(si);

            // Send earthquake data to API with real duration
            safePrint("🌋 Sending earthquake data to API - Magnitude: " + String(magnitude, 1) +
                     ", Duration: " + String(earthquakeDurationMs/1000.0, 1) + " seconds");
            sendEarthquakeToAPI(magnitude, earthquakeDurationMs);

            // Sound the success tone to indicate alert cleared
            soundSuccessTone();

            earthquakeActive = false;
            emailSent = false; // Reset for next earthquake
            earthquakeTriggeredOnce = false; // Reset so another earthquake can be detected

            safePrint("✅ EARTHQUAKE DISPLAY CLEARED after " + String(alertDuration/1000) + " seconds");
            safePrint("🔄 System returned to normal monitoring");
        }
    }
}

// ------------------- BUZZER FUNCTIONS -----------------
// Function to generate tone using analogWrite for better sound quality
void tone(uint8_t pin, unsigned int frequency, unsigned long duration) {
  unsigned long period = 1000000L / frequency;
  unsigned long elapsed = 0;

  while (elapsed < duration * 1000) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(pin, LOW);
    delayMicroseconds(period / 2);
    elapsed += period;
  }
}

// Function to sound the buzzer in a simple beeping alarm pattern
void soundEarthquakeAlarm() {
    safePrint("🔊 Playing earthquake alarm beep pattern");

    // Reduced number of beeps to prevent excessive buzzer activation
    for (int i = 0; i < 5; i++) {  // Reduced from 10 to 5 iterations
        // Fast alternating beep pattern
        tone(BUZZER_PIN, 2000, 200); // Higher pitch beep
        delay(200);  // Short pause
        tone(BUZZER_PIN, 2500, 200); // Higher pitch beep
        delay(200);  // Short pause
    }
}

// Function for a success tone
void soundSuccessTone() {
    safePrint("🔊 Playing success tone");
    // Three-note ascending success pattern
    tone(BUZZER_PIN, 1500, 100);
    delay(100);
    tone(BUZZER_PIN, 2000, 100);
    delay(100);
    tone(BUZZER_PIN, 3000, 200);
}



// ------------------- VALIDATE SENSOR READING ----------
bool isValidReading(float si, float pga) {
    // Check if readings are NaN, infinite, or unreasonable values
    if (isnan(si) || isnan(pga) || isinf(si) || isinf(pga)) {
        return false;
    }

    // Check for unreasonable values (adjust ranges as needed for your sensor)
    if (si < 0 || si > 100 || pga < 0 || pga > 100) {
        return false;
    }

    return true;
}

// ------------------- HANDLE SENSOR READINGS ------------
// Instead of detecting stuck readings, this function now just tracks the last reading
// and returns false (meaning not stuck) to allow normal operation
bool isReadingStuck(float currentSI) {
    // Only log when reading changes significantly (more than 0.1)
    if (abs(currentSI - lastSensorReading) > 0.1) {
        safePrint("  Reading changed from " + String(lastSensorReading, 3) + " to " + String(currentSI, 3));
    }

    // Store current reading for next comparison
    lastSensorReading = currentSI;

    // Never mark as stuck, always allow readings to be processed
    return false;
}

// ------------------- LOOP (SENSOR ONLY - NO SIMULATION) -----------------------------
void loop() {
    float si = 0.0, pga = 0.0;
    bool validReading = false;

    // Only process if sensor is ready
    if (sensorReady) {
        // Try to read from sensor
        si = D7S.getInstantaneusSI();
        pga = D7S.getInstantaneusPGA();

        // Check if sensor is stuck
        isReadingStuck(si);

        // Validate sensor readings
        if (!isValidReading(si, pga)) {
            consecutiveInvalidReadings++;
            safePrint("⚠ Invalid sensor reading #" + String(consecutiveInvalidReadings) + " - SI: " + String(si) + ", PGA: " + String(pga));

            // If too many consecutive invalid readings, mark sensor as not ready
            if (consecutiveInvalidReadings >= maxInvalidReadings) {
                safePrint("❌ Too many invalid readings, sensor marked as not ready");
                sensorReady = false;
                consecutiveInvalidReadings = 0;
                stuckReadingCount = 0;
                sensorStuck = false;
                earthquakeTriggeredOnce = false;
            }
        } else {
            // Valid sensor reading
            consecutiveInvalidReadings = 0;
            validReading = true;
        }
    } else {
        // Try to reinitialize sensor periodically
        if (millis() - lastSensorRetry >= sensorRetryInterval) {
            safePrint("🔧 Attempting sensor reconnection...");
            scanI2C();
            if (initializeSensor()) {
                sensorReady = true;
                consecutiveInvalidReadings = 0;
                stuckReadingCount = 0;
                lastSensorReading = -1;
                sensorStuck = false;
                earthquakeTriggeredOnce = false;
                stuckValue = -1;
                safePrint("✅ Sensor reconnected successfully!");
            }
            lastSensorRetry = millis();
        }
    }

    // Only process valid readings
    if (validReading) {
        // Add reading to circular buffer
        addReading(si);

        // Detect earthquakes for display/logging purposes AND EMAIL SENDING
        detectEarthquakeForDisplay(si);
    }

    // Print current reading every 20th loop (about 1 minute)
    static int loopCount = 0;
    loopCount++;
    if (loopCount % 20 == 0) {
        if (validReading || sensorReady) {
            float avg, minVal, maxVal;
            calculateStats(avg, minVal, maxVal);
            float magnitude = convertSIToMagnitude(si);

            String status = earthquakeActive ? "🚨 EARTHQUAKE ACTIVE" : "📊 Normal";
            String sourceStatus;

            if (!sensorReady) {
                sourceStatus = "NO SENSOR";
            } else {
                sourceStatus = "REAL";
            }
            // Removed stuck sensor display logic

            // EMAIL STATUS - UPDATED for no cooldown
            String emailStatus = emailSent ? " | Email: SENT" : " | Email: READY";
            // DISABLED: Cooldown check removed
            // if (millis() - lastEmailTime < emailCooldown && lastEmailTime > 0) {
            //     unsigned long cooldownRemaining = (emailCooldown - (millis() - lastEmailTime)) / 1000;
            //     emailStatus = " | Email: COOLDOWN(" + String(cooldownRemaining) + "s)";
            // }

            if (sensorReady) {
                safePrint(status + " | SI: " + String(si, 3) +
                         " | Magnitude: M" + String(magnitude, 1) +
                         " | Rolling Avg: " + String(avg, 3) +
                         " | Min: " + String(minVal, 3) +
                         " | Max: " + String(maxVal, 3) +
                         " | Source: " + sourceStatus + emailStatus);
            } else {
                safePrint("⚠️ WAITING FOR SENSOR | No readings available | Source: " + sourceStatus + emailStatus);
            }
        } else {
            safePrint("⚠️ WAITING FOR SENSOR | Attempting reconnection...");
        }
    }

    // Hourly buzzer test to ensure it's working
    if (millis() - lastBuzzerTest >= buzzerTestInterval) {
        safePrint("🔊 Running periodic buzzer health check...");
        tone(BUZZER_PIN, 2200, 200);
        lastBuzzerTest = millis();
        safePrint("🔊 Buzzer health check complete");
    }

    // 5-minute reporting (ONLY if we have valid data)
    if (millis() - lastAverageTime >= averageInterval) {
        if (totalReadings > 0) {
            float avgSI, minSI, maxSI;
            calculateStats(avgSI, minSI, maxSI);

            safePrint("\n📊 === 5-MINUTE REPORT ===");
            safePrint("Total readings: " + String(totalReadings));
            safePrint("Average SI: " + String(avgSI, 3));
            safePrint("Minimum SI: " + String(minSI, 3));
            safePrint("Maximum SI: " + String(maxSI, 3));
            safePrint("Earthquake status: " + String(earthquakeActive ? "ACTIVE (display only)" : "INACTIVE"));
            safePrint("Data source: " + String(sensorReady ? "Hardware sensor" : "No sensor"));
            safePrint("Invalid readings in period: " + String(consecutiveInvalidReadings));
            safePrint("Stuck reading detection disabled - all readings processed");

            // Send 5-minute statistical data
            sendReadingToAPI(avgSI, minSI, maxSI);
        } else {
            safePrint("\n📊 === 5-MINUTE REPORT ===");
            safePrint("⚠️ NO DATA COLLECTED - Sensor not ready or no valid readings");
            safePrint("Sensor status: " + String(sensorReady ? "Ready but no valid data" : "Not ready"));
        }

        batteryLevel -= 0.1;
        if (batteryLevel < 0) batteryLevel = 0;

        // Reset for next 5-minute window
        for (int i = 0; i < 100; i++) {
            readings[i] = 0.0;
        }
        readingIndex = 0;
        totalReadings = 0;
        lastAverageTime = millis();

        safePrint("========================\n");
    }

    delay(3000); // 3-second delay
}
