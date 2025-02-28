#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MS5837.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

// WiFi Configuration
const char* ssid = "G2";
const char* password = "23456789";

// Webserver
WebServer server(80);

// กำหนดพิน
#define I2C_SDA 21
#define I2C_SCL 22
#define MOTOR_FRONT_RIGHT 15
#define MOTOR_FRONT_LEFT 12
#define MOTOR_BACK_RIGHT 14
#define MOTOR_BACK_LEFT 26
#define SAFETY_CUTOFF_PIN 17  // เพิ่มพิน safety cutoff

#define MIN_THROTTLE 1100
#define MAX_THROTTLE 1900
#define BASE_THROTTLE 1510
#define STOP_THROTTLE 1505

struct LogData {
    float angleY;
    float angleZ;
    float depth;
    float kP_Y;
    float kI_Y;
    float kD_Y;
    float kP_Z;
    float kI_Z;
    float kD_Z;
    float kP_D;
    float kI_D;
    float kD_D;
    float targetDepth;
    unsigned long timestamp;
    bool safetyCutoffActive;  // เพิ่มสถานะ safety cutoff
};

// เพิ่มตัวแปรควบคุมสถานะการทำงาน
bool isRunning = false;
bool safetyCutoffActive = false;  // สถานะ safety cutoff
float targetDepth = 0.0;  // Target depth in meters
#define TARGET_ANGLE_Y 0.0
#define TARGET_ANGLE_Z 0.0

// ค่า PID สำหรับแกน Y และ Z
float kP_Y = 6.0;
float kI_Y = 0.0;
float kD_Y = 1.0;

float kP_Z = 6.0;
float kI_Z = 0.0;
float kD_Z = 1.0;

float kP_D = 6.0;  // Depth P gain
float kI_D = 0.0;  // Depth I gain
float kD_D = 1.0;  // Depth D gain

// Multithreading variables
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t webServerTaskHandle = NULL;
TaskHandle_t safetyCutoffTaskHandle = NULL;
SemaphoreHandle_t pidParamsMutex = NULL;
SemaphoreHandle_t sensorDataMutex = NULL;
SemaphoreHandle_t safetyCutoffMutex = NULL;

// สำหรับเก็บข้อมูลเซ็นเซอร์ที่ใช้ร่วมกันระหว่าง thread
struct SensorData {
    float angleY;
    float angleZ;
    float depth;
    unsigned long timestamp;
    bool safetyCutoffActive;  // เพิ่มสถานะ safety cutoff
} sensorData;

// โครงสร้างข้อมูลสำหรับค่า PID
struct PIDOutput {
    float frontRight;
    float frontLeft;
    float backRight;
    float backLeft;
    float verticalThrust;  // Added for depth control
    
    PIDOutput() : frontRight(0), frontLeft(0), backRight(0), backLeft(0), verticalThrust(0) {}
    PIDOutput(float fr, float fl, float br, float bl)
        : frontRight(fr), frontLeft(fl), backRight(br), backLeft(bl), verticalThrust(0) {}
};

// ตัวแปร PID
float errorY_prev = 0;
float errorZ_prev = 0;
float errorD_prev = 0;  // Previous depth error
float integralY = 0;
float integralZ = 0;
float integralD = 0;    // Depth integral
unsigned long lastTime = 0;

// อุปกรณ์
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
MS5837 depthSensor;
Servo motor_FR, motor_FL, motor_BR, motor_BL;

void setupWiFi() {
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

TaskHandle_t dataLoggerTaskHandle = NULL;

// Web Server Handler Functions
void handleControl() {
    String action = server.arg("action");
    
    if (action == "start") {
        isRunning = true;
        server.send(200, "text/plain", "Started");
    } else if (action == "stop") {
        isRunning = false;
        // มอเตอร์จะถูกหยุดในลูปควบคุม
        server.send(200, "text/plain", "Stopped");
    }
}

void handleUpdate() {
    // เข้าถึงพารามิเตอร์ PID โดยใช้ mutex เพื่อป้องกันการอ่าน/เขียนพร้อมกัน
    if (xSemaphoreTake(pidParamsMutex, portMAX_DELAY) == pdTRUE) {
        if (server.hasArg("kp_y")) {
            kP_Y = server.arg("kp_y").toFloat();
        }
        if (server.hasArg("ki_y")) {
            kI_Y = server.arg("ki_y").toFloat();
        }
        if (server.hasArg("kd_y")) {
            kD_Y = server.arg("kd_y").toFloat();
        }
        if (server.hasArg("kp_z")) {
            kP_Z = server.arg("kp_z").toFloat();
        }
        if (server.hasArg("ki_z")) {
            kI_Z = server.arg("ki_z").toFloat();
        }
        if (server.hasArg("kd_z")) {
            kD_Z = server.arg("kd_z").toFloat();
        }
        if (server.hasArg("kp_D")) {
            kP_D = server.arg("kp_D").toFloat();
        }
        if (server.hasArg("ki_D")) {
            kI_D = server.arg("ki_D").toFloat();
        }
        if (server.hasArg("kd_D")) {
            kD_D = server.arg("kd_D").toFloat();
        }
        if (server.hasArg("target_depth")) {
            targetDepth = server.arg("target_depth").toFloat();
        }
        
        xSemaphoreGive(pidParamsMutex);
    }
    
    // อ่านค่าเซนเซอร์ล่าสุดสำหรับแสดงผล
    float currentAngleY = 0, currentAngleZ = 0, currentDepth = 0;
    bool currentSafetyCutoff = false;
    
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
        currentAngleY = sensorData.angleY;
        currentAngleZ = sensorData.angleZ;
        currentDepth = sensorData.depth;
        currentSafetyCutoff = sensorData.safetyCutoffActive;
        xSemaphoreGive(sensorDataMutex);
    }
    
    String html = "<!DOCTYPE html>"
                 "<html><head>"
                 "<title>AUV PID Tuner</title>"
                 "<style>"
                 ".control-buttons { margin: 20px 0; }"
                 ".control-buttons button { padding: 10px 20px; margin: 0 10px; cursor: pointer; }"
                 ".start-btn { background-color: #4CAF50; color: white; border: none; }"
                 ".stop-btn { background-color: #f44336; color: white; border: none; }"
                 ".sensor-data { background-color: #f0f0f0; padding: 10px; margin: 20px 0; }"
                 ".safety-active { background-color: #f44336; color: white; padding: 5px; }"
                 ".safety-inactive { background-color: #4CAF50; color: white; padding: 5px; }"
                 "</style>"
                 "<script>"
                 "function controlQuad(action) {"
                 "  fetch('/control?action=' + action) "
                 "    .then(response => response.text())"
                 "    .then(data => alert('Command: ' + action + ' - ' + data));"
                 "} "
                 "function refreshSensorData() {"
                 "  fetch('/sensordata')"
                 "    .then(response => response.json())"
                 "    .then(data => {"
                 "      document.getElementById('angleY').textContent = data.angleY.toFixed(2);"
                 "      document.getElementById('angleZ').textContent = data.angleZ.toFixed(2);"
                 "      document.getElementById('depth').textContent = data.depth.toFixed(2);"
                 "      const safetyElem = document.getElementById('safetyCutoff');"
                 "      safetyElem.textContent = data.safetyCutoffActive ? 'ACTIVE' : 'INACTIVE';"
                 "      safetyElem.className = data.safetyCutoffActive ? 'safety-active' : 'safety-inactive';"
                 "    });"
                 "  setTimeout(refreshSensorData, 1000);"
                 "}"
                 "document.addEventListener('DOMContentLoaded', refreshSensorData);"
                 "</script>"
                 "</head>"
                 "<body>"
                 "<h1>AUV PID Parameter Tuner</h1>"
                 "<div class='control-buttons'>"
                 "<button class='start-btn' onclick='controlQuad(\"start\")'>Start</button>"
                 "<button class='stop-btn' onclick='controlQuad(\"stop\")'>Stop</button>"
                 "</div>"
                 "<div class='sensor-data'>"
                 "<h2>Current Sensor Data:</h2>"
                 "<p>Angle Y (Pitch): <span id='angleY'>" + String(currentAngleY, 2) + "</span></p>"
                 "<p>Angle Z (Roll): <span id='angleZ'>" + String(currentAngleZ, 2) + "</span></p>"
                 "<p>Depth: <span id='depth'>" + String(currentDepth, 2) + "</span> m</p>"
                 "<p>Safety Cutoff: <span id='safetyCutoff' class='" + 
                 (currentSafetyCutoff ? "safety-active" : "safety-inactive") + "'>" + 
                 (currentSafetyCutoff ? "ACTIVE" : "INACTIVE") + "</span></p>"
                 "</div>"
                 "<form action='/update' method='GET'>"
                 "<h2>Y Axis (Pitch)</h2>"
                 "Kp_Y: <input type='text' name='kp_y' value='" + String(kP_Y) + "'><br>"
                 "Ki_Y: <input type='text' name='ki_y' value='" + String(kI_Y) + "'><br>"
                 "Kd_Y: <input type='text' name='kd_y' value='" + String(kD_Y) + "'><br>"
                 "<h2>Z Axis (Roll)</h2>"
                 "Kp_Z: <input type='text' name='kp_z' value='" + String(kP_Z) + "'><br>"
                 "Ki_Z: <input type='text' name='ki_z' value='" + String(kI_Z) + "'><br>"
                 "Kd_Z: <input type='text' name='kd_z' value='" + String(kD_Z) + "'><br>"
                 "<h2>Depth</h2>"
                 "Kp_D: <input type='text' name='kp_D' value='" + String(kP_D) + "'><br>"
                 "Ki_D: <input type='text' name='ki_D' value='" + String(kI_D) + "'><br>"
                 "Kd_D: <input type='text' name='kd_D' value='" + String(kD_D) + "'><br>"
                 "<h2>Target Depth</h2>"
                 "Target Depth: <input type='text' name='target_depth' value='" + String(targetDepth) + "'><br>"
                 "<input type='submit' value='Update PID'>"
                 "</form>"
                 "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleRoot() {
    server.sendHeader("Location", "/update", true);
    server.send(302, "text/plain", "");
}

void handleSensorData() {
    float currentAngleY = 0, currentAngleZ = 0, currentDepth = 0;
    bool currentSafetyCutoff = false;
    
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
        currentAngleY = sensorData.angleY;
        currentAngleZ = sensorData.angleZ;
        currentDepth = sensorData.depth;
        currentSafetyCutoff = sensorData.safetyCutoffActive;
        xSemaphoreGive(sensorDataMutex);
    }
    
    String jsonResponse = "{\"angleY\":" + String(currentAngleY, 2) + 
                         ",\"angleZ\":" + String(currentAngleZ, 2) + 
                         ",\"depth\":" + String(currentDepth, 2) + 
                         ",\"safetyCutoffActive\":" + (currentSafetyCutoff ? "true" : "false") + "}";
    
    server.send(200, "application/json", jsonResponse);
}

// ฟังก์ชัน calculatePID สำหรับคำนวณค่าควบคุม
PIDOutput calculatePID(float angleY, float angleZ, float depth) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt == 0) return PIDOutput();
    
    float local_kP_Y, local_kI_Y, local_kD_Y;
    float local_kP_Z, local_kI_Z, local_kD_Z;
    float local_kP_D, local_kI_D, local_kD_D;
    float local_targetDepth;
    
    // อ่านค่า PID parameters ด้วย mutex
    if (xSemaphoreTake(pidParamsMutex, portMAX_DELAY) == pdTRUE) {
        local_kP_Y = kP_Y; local_kI_Y = kI_Y; local_kD_Y = kD_Y;
        local_kP_Z = kP_Z; local_kI_Z = kI_Z; local_kD_Z = kD_Z;
        local_kP_D = kP_D; local_kI_D = kI_D; local_kD_D = kD_D;
        local_targetDepth = targetDepth;
        xSemaphoreGive(pidParamsMutex);
    }
    
    // Calculate orientation errors
    float errorY = TARGET_ANGLE_Y - angleY;
    float errorZ = TARGET_ANGLE_Z - angleZ;
    float errorD = (local_targetDepth - depth) * 100.0;
    
    // Calculate integrals
    integralY += errorY * dt;
    integralZ += errorZ * dt;
    integralD += errorD * dt;
    
    // Constrain integrals
    integralY = constrain(integralY, -100, 100);
    integralZ = constrain(integralZ, -100, 100);
    integralD = constrain(integralD, -100, 100);
    
    // Calculate derivatives
    float derivativeY = (errorY - errorY_prev) / dt;
    float derivativeZ = (errorZ - errorZ_prev) / dt;
    float derivativeD = (errorD - errorD_prev) / dt;
    
    // Calculate PID outputs
    float outputY = local_kP_Y * errorY + local_kI_Y * integralY + local_kD_Y * derivativeY;
    float outputZ = local_kP_Z * errorZ + local_kI_Z * integralZ + local_kD_Z * derivativeZ;
    float outputD = local_kP_D * errorD + local_kI_D * integralD + local_kD_D * derivativeD;
    
    // Update previous errors
    errorY_prev = errorY;
    errorZ_prev = errorZ;
    errorD_prev = errorD;
    lastTime = now;
    
    // Calculate motor outputs including depth control
    PIDOutput output(
        constrain(BASE_THROTTLE - outputY - outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE - outputY + outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY - outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY + outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE)
    );
    
    return output;
}

// ฟังก์ชัน Task สำหรับการตรวจสอบ Safety Cutoff
void safetyCutoffTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // ตั้งความถี่ต่ำกว่า control task (ทุก 100ms หรือ 10Hz)
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    
    bool previousCutoffState = false;
    
    while(true) {
        // ตรวจสอบสถานะ safety cutoff
        bool currentCutoffState = digitalRead(SAFETY_CUTOFF_PIN) == LOW;
        
        // ป้องกันการเข้าถึงตัวแปรร่วมพร้อมกัน
        if (xSemaphoreTake(safetyCutoffMutex, portMAX_DELAY) == pdTRUE) {
            safetyCutoffActive = currentCutoffState;
            xSemaphoreGive(safetyCutoffMutex);
        }
        
        // แสดงข้อความเฉพาะเมื่อสถานะเปลี่ยน
        if (currentCutoffState != previousCutoffState && currentCutoffState) {
            Serial.println("SAFETY CUTOFF ACTIVATED - MOTORS WILL BE STOPPED");
        } else if (currentCutoffState != previousCutoffState && !currentCutoffState) {
            Serial.println("SAFETY CUTOFF DEACTIVATED");
        }
        
        previousCutoffState = currentCutoffState;
        
        // ทำงานที่ความถี่ที่กำหนด
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ปรับแก้ controlTask โดยลบการตรวจสอบ safety cutoff ออก
void controlTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz control loop
    
    while(true) {
        // อ่านค่าเซ็นเซอร์
        sensors_event_t event;
        bno.getEvent(&event);
        depthSensor.read();
        
        float currentDepth = depthSensor.depth();
        float currentAngleY = event.orientation.y;
        float currentAngleZ = event.orientation.z;
        
        // อัพเดทข้อมูลเซ็นเซอร์สำหรับ webserver
        bool localSafetyCutoffActive = false;
        if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
            sensorData.angleY = currentAngleY;
            sensorData.angleZ = currentAngleZ;
            sensorData.depth = currentDepth;
            sensorData.timestamp = millis();
            
            // อ่านค่า safetyCutoffActive ที่อัปเดตโดย safetyCutoffTask
            if (xSemaphoreTake(safetyCutoffMutex, portMAX_DELAY) == pdTRUE) {
                localSafetyCutoffActive = safetyCutoffActive;
                sensorData.safetyCutoffActive = safetyCutoffActive;
                xSemaphoreGive(safetyCutoffMutex);
            }
            
            xSemaphoreGive(sensorDataMutex);
        }
        
        // ถ้า safety cutoff ทำงานให้หยุดมอเตอร์ทันที
        if (localSafetyCutoffActive) {
            motor_FR.writeMicroseconds(STOP_THROTTLE);
            motor_FL.writeMicroseconds(STOP_THROTTLE);
            motor_BR.writeMicroseconds(STOP_THROTTLE);
            motor_BL.writeMicroseconds(STOP_THROTTLE);
        }
        // ถ้า safety cutoff ไม่ทำงาน และ isRunning เป็น true ให้ทำงานปกติ
        else if (isRunning) {
            PIDOutput motorSpeeds = calculatePID(currentAngleY, currentAngleZ, currentDepth);
            
            // ส่งค่าควบคุมไปยังมอเตอร์
            motor_FR.writeMicroseconds(motorSpeeds.frontRight);
            motor_FL.writeMicroseconds(motorSpeeds.frontLeft);
            motor_BR.writeMicroseconds(motorSpeeds.backRight);
            motor_BL.writeMicroseconds(motorSpeeds.backLeft);
            Serial.print(motorSpeeds.frontRight);
            Serial.print(", ");
            Serial.print(motorSpeeds.frontLeft);
            Serial.print(", ");
            Serial.print(motorSpeeds.backRight);
            Serial.print(", ");
            Serial.print(motorSpeeds.backLeft);
            Serial.println(".");
        } else {
            // ถ้าไม่ได้ทำงานให้หยุดมอเตอร์
            motor_FR.writeMicroseconds(STOP_THROTTLE);
            motor_FL.writeMicroseconds(STOP_THROTTLE);
            motor_BR.writeMicroseconds(STOP_THROTTLE);
            motor_BL.writeMicroseconds(STOP_THROTTLE);
        }
        
        // ทำงานแบบเวลาแน่นอน
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task สำหรับการจัดการ webserver
void webServerTask(void *parameter) {
    while(true) {
        server.handleClient();
        // ให้เวลา task อื่นๆ ทำงาน
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void dataLoggerTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz logging rate
    
    while(true) {
        LogData data;
        
        // Get sensor data
        if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
            data.angleY = sensorData.angleY;
            data.angleZ = sensorData.angleZ;
            data.depth = sensorData.depth;
            data.timestamp = sensorData.timestamp;
            data.safetyCutoffActive = sensorData.safetyCutoffActive;
            xSemaphoreGive(sensorDataMutex);
        }
        
        // Get PID parameters
        if (xSemaphoreTake(pidParamsMutex, portMAX_DELAY) == pdTRUE) {
            data.kP_Y = kP_Y;
            data.kI_Y = kI_Y;
            data.kD_Y = kD_Y;
            data.kP_Z = kP_Z;
            data.kI_Z = kI_Z;
            data.kD_Z = kD_Z;
            data.kP_D = kP_D;
            data.kI_D = kI_D;
            data.kD_D = kD_D;
            data.targetDepth = targetDepth;
            xSemaphoreGive(pidParamsMutex);
        }
        
        // Send data in CSV format
        Serial.printf("DATA,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu\n",
            data.angleY, data.angleZ, data.depth,
            data.kP_Y, data.kI_Y, data.kD_Y,
            data.kP_Z, data.kI_Z, data.kD_Z,
            data.kP_D, data.kI_D, data.kD_D,
            data.targetDepth, data.timestamp
        );
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); // ให้ Serial port พร้อมใช้งาน
    Serial.println("Starting AUV Control System");
    
    // สร้าง mutex สำหรับการเข้าถึงทรัพยากรร่วม
    pidParamsMutex = xSemaphoreCreateMutex();
    sensorDataMutex = xSemaphoreCreateMutex();
    safetyCutoffMutex = xSemaphoreCreateMutex(); // เพิ่ม mutex สำหรับ safety cutoff
    
    // Setup I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // ตั้งค่า safety cutoff pin
    pinMode(SAFETY_CUTOFF_PIN, INPUT);
    Serial.println("Safety cutoff system initialized on pin 17");
    
    // ตั้งค่า WiFi
    setupWiFi();

    // ตั้งค่า Web Server
    server.on("/", handleRoot);
    server.on("/update", handleUpdate);
    server.on("/control", handleControl);
    server.on("/sensordata", handleSensorData);
    server.begin();
    Serial.println("Web server started");
    
    // ตั้งค่า BNO055
    if(!bno.begin()) {
        Serial.println("BNO055 not detected!");
        while(1);
    }
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized");

    // ตั้งค่า MS5837
    if (!depthSensor.init()) {
        Serial.println("MS5837 not detected!");
        while(1);
    }
    depthSensor.setModel(MS5837::MS5837_30BA);
    depthSensor.setFluidDensity(997); // kg/m^3 for freshwater
    Serial.println("MS5837 initialized");
    
    // ตั้งค่า ESC
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    motor_FR.setPeriodHertz(50);
    motor_FL.setPeriodHertz(50);
    motor_BR.setPeriodHertz(50);
    motor_BL.setPeriodHertz(50);
    
    motor_FR.attach(MOTOR_FRONT_RIGHT, MIN_THROTTLE, MAX_THROTTLE);
    motor_FL.attach(MOTOR_FRONT_LEFT, MIN_THROTTLE, MAX_THROTTLE);
    motor_BR.attach(MOTOR_BACK_RIGHT, MIN_THROTTLE, MAX_THROTTLE);
    motor_BL.attach(MOTOR_BACK_LEFT, MIN_THROTTLE, MAX_THROTTLE);
    
    // ตั้งค่าเริ่มต้นมอเตอร์
    motor_FR.writeMicroseconds(STOP_THROTTLE);
    motor_FL.writeMicroseconds(STOP_THROTTLE);
    motor_BR.writeMicroseconds(STOP_THROTTLE);
    motor_BL.writeMicroseconds(STOP_THROTTLE);
    delay(2000);
    Serial.println("ESCs initialized");
    
    // สร้าง tasks
    xTaskCreatePinnedToCore(
        controlTask,       // Task function
        "ControlTask",     // Task name
        8192,              // Stack size
        NULL,              // Parameters
        3,                 // Priority (higher number = higher priority)
        &controlTaskHandle,// Task handle
        0                  // Core (0 or 1)
    );
    
    xTaskCreatePinnedToCore(
        webServerTask,     // Task function
        "WebServerTask",   // Task name
        8192,              // Stack size
        NULL,              // Parameters
        1,                 // Priority
        &webServerTaskHandle, // Task handle
        1                  // Core
    );
    
    xTaskCreatePinnedToCore(
        dataLoggerTask,    // Task function
        "DataLoggerTask",  // Task name
        8192,              // Stack size
        NULL,              // Parameters
        1,                 // Priority
        &dataLoggerTaskHandle, // Task handle
        1                  // Core
    );
    
    // เพิ่ม Safety Cutoff Task (ความสำคัญสูงกว่า task อื่นๆ)
    xTaskCreatePinnedToCore(
        safetyCutoffTask,  // Task function
        "SafetyCutoffTask",// Task name
        2048,              // Stack size (น้อยกว่าเพราะ task เบากว่า)
        NULL,              // Parameters
        2,                 // Priority (สูงกว่า control task)
        &safetyCutoffTaskHandle, // Task handle
        1                  // Core (ทำงานบน core เดียวกับ control task)
    );

    Serial.println("Tasks created and running");
    Serial.println("Control and SafetyCutoff tasks running on core 0, WebServer and DataLogger tasks running on core 1");
}

// ฟังก์ชัน loop จะไม่ถูกใช้งานเนื่องจากใช้ FreeRTOS tasks แทน
void loop() {
    // ไม่ต้องทำอะไรเพราะใช้ task แล้ว
    vTaskDelay(pdMS_TO_TICKS(1000));
}