#include <Wire.h>
#include <TinyGPSPlus.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <PulseSensorPlayground.h>

// WiFi Credentials
const char* ssid = "krishna";
const char* password = "123456789";

// ThingSpeak settings
unsigned long channelID = 2859280;
const char* writeAPIKey = "V7TNITRAM0HIOYMX";

WiFiClient client;

// GPS Module (NEO-6M) pins
#define GPS_RX 32  
#define GPS_TX 33  
TinyGPSPlus gps;

// MPU6050 (Accelerometer & Gyroscope)
MPU6050 mpu;
float prevAngleX = 0, prevAngleY = 0, prevAngleZ = 0;
const float GYRO_THRESHOLD = 30.0;  // Threshold for drastic angle change

// Pulse Sensor setup
const int pulsePin = 34;
PulseSensorPlayground pulseSensor;
int pulseRate = 0;

// IR Sensor Array Pins
const int irPins[5] = {13, 14, 25, 26, 27};
int irValues[5];

// Touch Sensor Pin (Helmet Detection)
const int touchPin = 4;
int touchState = 0;

// Accident Detection Thresholds
const float ACCIDENT_THRESHOLD = 2.0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Trying to connect to WiFi...");
    }
    Serial.println("Connected to WiFi");
    ThingSpeak.begin(client);
    
    Wire.begin(21, 22);
    mpu.initialize();
    if (!mpu.testConnection()) Serial.println("MPU6050 connection failed!");
    
    pulseSensor.analogInput(pulsePin);
    pulseSensor.setThreshold(600);
    if (!pulseSensor.begin()) Serial.println("Pulse Sensor failed to initialize!");
    
    for (int i = 0; i < 5; i++) pinMode(irPins[i], INPUT);
    pinMode(touchPin, INPUT);
    Serial.println("Sensors initialized.");
}

void loop() {
    // Read GPS Data
    while (Serial2.available() > 0) gps.encode(Serial2.read());
    float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    float lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    ThingSpeak.setField(1, lat);
    ThingSpeak.setField(2, lon);
    
    // Read MPU6050 Data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    
    float ax_g = ax / 16384.0, ay_g = ay / 16384.0, az_g = az / 16384.0;
    float acceleration = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    ThingSpeak.setField(3, acceleration);
    
    float angleX = gx / 131.0, angleY = gy / 131.0, angleZ = gz / 131.0;
    bool severeGyroChange = (abs(angleX - prevAngleX) > GYRO_THRESHOLD || abs(angleY - prevAngleY) > GYRO_THRESHOLD || abs(angleZ - prevAngleZ) > GYRO_THRESHOLD);
    prevAngleX = angleX, prevAngleY = angleY, prevAngleZ = angleZ;
    
    // Read Pulse Sensor Data
    if (pulseSensor.sawStartOfBeat()) {
        pulseRate = pulseSensor.getBeatsPerMinute();
    }
    ThingSpeak.setField(4, pulseRate);
    
    // Read IR Sensor Data
    bool abnormalIR = false;
    for (int i = 0; i < 5; i++) {
        irValues[i] = digitalRead(irPins[i]);
        if (irValues[i] == HIGH) abnormalIR = true;
    }

    // Read Touch Sensor Data (Helmet Detection)
    touchState = digitalRead(touchPin);
    ThingSpeak.setField(5, touchState);
    
    Serial.println("=============================");
    Serial.printf("Latitude: %.6f\nLongitude: %.6f\n", lat, lon);
    Serial.printf("Pulse Rate (BPM): %d\n", pulseRate);
    Serial.printf("Acceleration (g): %.2f\n", acceleration);
    Serial.printf("Gyro Change: X=%.2f, Y=%.2f, Z=%.2f\n", angleX, angleY, angleZ);
    Serial.print("IR Sensor States: ");
    for (int i = 0; i < 5; i++) Serial.print(irValues[i]);
    Serial.println();
    Serial.printf("Touch Sensor State: %s\n", touchState ? "Helmet Worn" : "Helmet Not Worn");

    // Helmet Detection Condition
    if (touchState == LOW) {  // Helmet not worn
        Serial.println("!!! WARNING: Helmet Not Worn !!!");
        ThingSpeak.setField(6, 2);  // 2 indicates no helmet
    } else {
        // Accident Detection Logic
        bool accidentDetected = (acceleration > ACCIDENT_THRESHOLD) || (pulseRate > 100) || severeGyroChange || abnormalIR;
        if (accidentDetected) {
            Serial.println("Rider is driving safely.");
            ThingSpeak.setField(6, 1);  // 1 indicates an accident
        } else {
            Serial.println("!!! ACCIDENT DETECTED !!!");
            ThingSpeak.setField(6, 0);  // 0 indicates safe driving
        }
    }

    Serial.println("=============================");
    
    // Send Data to ThingSpeak with Increased Delay
    if (ThingSpeak.writeFields(channelID, writeAPIKey) == 200) {
        Serial.println("Data sent to ThingSpeak successfully!");
    } else {
        Serial.println("Failed to send data to ThingSpeak. Retrying...");
        delay(5000);
        ThingSpeak.writeFields(channelID, writeAPIKey);
    }

    delay(5000);  // Increased delay for ThingSpeak
}