#include <Losant.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "esp_sleep.h"
#include <HTTPClient.h>


// WiFi network settings
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

//information from the censor globals
int distance_mm = 0; // Millimeters for the laser sensor
int distance_cm = 0; // Centimeters for the ultrasonic sensor
int distance_laser_cm = 0; //converted laser to cm

// Pin definitions
const int LED_PIN = 13;
const int TRIG_PIN = 25;
const int ECHO_PIN = 26;

// Losant credentials
const char* LOSANT_DEVICE_ID = "";
const char* LOSANT_ACCESS_KEY = "";
const char* LOSANT_ACCESS_SECRET = "";
// DigiCert Global Root CA  https://www.digicert.com/kb/digicert-root-certificates.htm https://forums.losant.com/t/solved-losant-brokers-mosquitto-dependent-on-losant-losantrootca-crt-are-all-unable-to-connect-to-losant/1801/2
const char* rootCABuff = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
"-----END CERTIFICATE-----\n";

// Sensor instance
VL53L0X sensor;

// WiFi and Losant clients
WiFiClientSecure wifiClient;
LosantDevice device(LOSANT_DEVICE_ID);


void setup() {
    Serial.begin(115200);
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("Wake up from timer. Doing minimal reinitialization.");
    } else {
        Serial.println("Normal startup or not a deep sleep reset.");
    }

    // Setting up pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize I2C, WiFi, and sensors
    Wire.begin();  // Initialize I2C for the VL53L0X
    connectToWiFi();
    initSensor();

    // Connect to Losant
    connectToLosant();
}

void initSensor() {
    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println("Sensor initialization failed. Attempting retry...");
        delay(1000);
        if (!sensor.init()) {
            Serial.println("Sensor failed to initialize after retry.");
        } else {
            Serial.println("Sensor initialized successfully after retry.");
        }
    } else {
        Serial.println("Sensor initialized successfully on first attempt.");
    }
}

void loop() {
    handleLED();
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
    if (!device.connected()) {
        connectToLosant();
    }
    measureDistance();
    sendData(); //losant
    sendDataToGoogleCloud();
    maintainConnections();
    goToSleep();
}

void connectToWiFi() {
    WiFi.mode(WIFI_OFF); // Ensure WiFi is fully turned off
    delay(100); // Allow some time for the WiFi hardware to reset
    WiFi.mode(WIFI_STA); // Set mode to station

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("Attempting to connect to WiFi...");

    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < 20) { // Attempt to connect multiple times
        delay(500);
        Serial.print(".");
        retryCount++;
        if (retryCount == 10) {
            WiFi.disconnect();
            delay(1000); // Allow the disconnection process to complete
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Try reconnecting again
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected successfully!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect to WiFi.");
        // Additional logic to handle WiFi connection failure
    }
}

void connectToLosant() {
  wifiClient.setCACert(rootCABuff); // Assuming rootCABuff is defined elsewhere
  device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
  while(!device.connected()) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to Losant");
}

//Additional functionality, but this will result in 404 due to no longer us hosting the server
void sendDataToGoogleCloud() {
    HTTPClient http;
    StaticJsonDocument<200> doc;
    JsonObject root = doc.to<JsonObject>();

    root["laserDist"] = distance_mm;
    root["ultrasonicDist"] = distance_cm;

    String jsonObject;
    serializeJson(root, jsonObject);

    String serverPath = "https://REGION-PROJECT_ID.cloudfunctions.net/FUNCTION_NAME";  // this is a mockup address for google clou functions
    //real server not implemented due to costs but this should work for an open cloud functions.

    http.begin(serverPath);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonObject);

    if (httpResponseCode > 0) {
        String payload = http.getString();

        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.print("Response: ");
        Serial.println(payload);

        if (payload.equals("stop")) {
            Serial.println("Received 'stop' command. Shutting down...");
            esp_deep_sleep_start();
        }
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }

    http.end();
}

void handleLED() {
  digitalWrite(LED_PIN, LOW); // Turn the LED on
  delay(1000);
  digitalWrite(LED_PIN, HIGH); // Turn the LED off
  delay(2000);
}

void measureDistance() {
    // Only attempt to measure distance if sensor is initialized
    if (sensor.timeoutOccurred()) {
        Serial.println("Sensor timeout, attempting re-initialization.");
        initSensor();
        if (!sensor.init()) {
            Serial.println("Sensor reinitialization failed.");
            return;
        }
    }

    // Laser distance measurement
    Serial.print("Distance Laser [cm]: ");
    distance_mm = sensor.readRangeSingleMillimeters();
    distance_laser_cm = distance_mm / 10;
    if (sensor.timeoutOccurred()) {
        Serial.println("TIMEOUT");
        distance_mm = -1;  // Indicate an error
    } else {
        Serial.println(distance_laser_cm);
    }

    // Ultrasonic distance measurement
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
    if (duration == 0) {
        Serial.println("Ultrasonic sensor timeout");
        distance_cm = -1;  // Indicate an error
        return;
    }
    distance_cm = duration * 0.034 / 2;
    Serial.print("Distance [cm]: ");
    Serial.println(distance_cm);
}

void maintainConnections() {
  if (WiFi.status() != WL_CONNECTED || !device.connected()) {
    connectToWiFi();
    connectToLosant();
  }
  device.loop();
}

void sendData() {
    StaticJsonDocument<200> doc;
    JsonObject root = doc.to<JsonObject>();  // Extract the JsonObject from the StaticJsonDocument.
    
    // Use actual measured values instead of random values
    root["laserDist"] = distance_laser_cm;
    root["ultrasonicDist"] = distance_cm;

    Serial.println("Sending data to Losant");
    device.sendState(root);  // Pass the JsonObject, not the Document.

    Serial.println("JsonObject to be sent:");
    serializeJson(root, Serial);  // Output the JSON to the serial monitor.
    Serial.println();
}

void goToSleep() {
  // Ensure all operations complete
  if (WiFi.status() == WL_CONNECTED && device.connected()) {
    Serial.println("All operations completed. Going to sleep for 0.5 seconds.");
    esp_sleep_enable_timer_wakeup(500000); // 0.5 seconds
    esp_deep_sleep_start();
  } else {
    Serial.println("Waiting for operations to complete before sleeping.");
  }
}
