#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// Pin definitions
#define RED_PIN 25
#define GREEN_PIN 26
#define BLUE_PIN 27
#define CUTOFF_PIN 17

// EEPROM addresses for PID values
#define EEPROM_SIZE 24
#define ADDR_KP_X 0
#define ADDR_KI_X 4
#define ADDR_KD_X 8
#define ADDR_KP_Y 12
#define ADDR_KI_Y 16
#define ADDR_KD_Y 20

// Default PID constants if EEPROM is not set
#define DEFAULT_KP_X 2.0
#define DEFAULT_KI_X 0.01
#define DEFAULT_KD_X 1.0
#define DEFAULT_KP_Y 2.0
#define DEFAULT_KI_Y 0.01
#define DEFAULT_KD_Y 1.0

// Motor neutral position (stop)
#define MOTOR_NEUTRAL 1505

// PID calculation interval (in milliseconds)
#define PID_INTERVAL 20

// Auto shutdown time for 777 code (5 minutes in milliseconds)
#define AUTO_SHUTDOWN_TIME 20000

// EN timeout check - 3 seconds in milliseconds
#define EN_TIMEOUT 3000

// WiFi credentials - you should change these to match your network
const char* ssid = "G2";
const char* password = "23456789";

// Maximum connection attempts
const int maxConnectionAttempts = 20;

// Task handles for multithreading
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;

// Mutex for protecting shared variables
SemaphoreHandle_t dataMutex;

WebServer server(80);

String receivedData;
int EX = 0, EY = 0, EN = 1;
int prevEX = -1, prevEY = -1, prevEN = -1;
int countEN = 0;

// Variables for EN timeout tracking
unsigned long lastENChangeTime = 0;
bool enTimeoutActive = false;

// Variables for 777 auto-shutdown feature
bool isInShutdownCountdown = false;
unsigned long shutdownStartTime = 0;

// PID variables
float Kp_X, Ki_X, Kd_X;
float Kp_Y, Ki_Y, Kd_Y;

float errorSumX = 0;
float lastErrorX = 0;
float outputX = 0;

float errorSumY = 0;
float lastErrorY = 0;
float outputY = 0;

// Motor pins
const int motor1Pin = 33;
const int motor2Pin = 32;

Servo motor1;
Servo motor2;

// Mode switching variables
const int CHECK_INTERVAL = 500;
const int CHECK_COUNT = 6;
unsigned long lastCheckTime = 0;
unsigned long lastPIDTime = 0;
int stableCount = 0;
int mode = 2;  // Default to stopped (mode 2)
bool isBlinkOn = false;

// Function declarations
void pidControlTask(void *parameter);
void webServerTask(void *parameter);
void loadPIDValues();
void savePIDValues();
void handleRoot();
void handleUpdate();
void handleNotFound();
void setColor(int red, int green, int blue);
float calculatePID_X(float error);
float calculatePID_Y(float error);
void parseErrorData(String data);

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Ready with Multithreaded PID Control and Web Interface!");
    
    // Create mutex for data protection
    dataMutex = xSemaphoreCreateMutex();
    
    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    
    // Load PID values from EEPROM or use defaults
    loadPIDValues();
    
    // Setup pins
    pinMode(CUTOFF_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    
    // Setup PWM timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Configure servo motors
    motor1.setPeriodHertz(50);
    motor2.setPeriodHertz(50);
    motor1.attach(motor1Pin, 1000, 2000);
    motor2.attach(motor2Pin, 1000, 2000);

    // Initialize motors to neutral position
    motor1.writeMicroseconds(MOTOR_NEUTRAL);
    motor2.writeMicroseconds(MOTOR_NEUTRAL);

    // Initialize the EN change time
    lastENChangeTime = millis();

    // Set RGB LED to blue to indicate connecting to WiFi
    setColor(0, 0, 255);

    // Setup WiFi client mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    // Wait for connection with timeout
    int attempts = 0;
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED && attempts < maxConnectionAttempts) {
        delay(500);
        Serial.print(".");
        attempts++;
        // Blink blue LED while connecting
        if (attempts % 2 == 0) {
            setColor(0, 0, 255);
        } else {
            setColor(0, 0, 0);
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.print("Connected to WiFi network with IP Address: ");
        Serial.println(WiFi.localIP());
        setColor(0, 255, 255); // Cyan light for successful connection
        delay(1000);
    } else {
        Serial.println("");
        Serial.println("Failed to connect to WiFi. Check credentials or network availability.");
        setColor(255, 0, 255); // Purple light for connection failure
        delay(3000);
        ESP.restart(); // Restart ESP32 to try again
    }

    // Create tasks for PID control and web server
    xTaskCreatePinnedToCore(
        pidControlTask,         // Task function
        "PIDControlTask",       // Name of task
        8192,                   // Stack size of task
        NULL,                   // Parameter of the task
        1,                      // Priority of the task (higher number = higher priority)
        &pidTaskHandle,         // Task handle to keep track of created task
        0);                     // Core where the task should run (0 = same as Arduino loop)
        
    xTaskCreatePinnedToCore(
        webServerTask,          // Task function
        "WebServerTask",        // Name of task
        8192,                   // Stack size of task
        NULL,                   // Parameter of the task
        1,                      // Priority of the task
        &webTaskHandle,         // Task handle
        1);                     // Run on core 1
        
    Serial.println("Multithreading started. PID Control on Core 0, Web Server on Core 1");

    // Initial delay to allow motors to settle
    delay(2000);
}

// Loop function isn't used since we moved everything to tasks
void loop() {
    // Nothing here - functionality moved to tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// Task for PID control, serial communication, and motor control
void pidControlTask(void *parameter) {
    for(;;) {
        // Take mutex before accessing shared data
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            // Read serial data if available
            if (Serial.available() > 0) {
                receivedData = Serial.readStringUntil('\n');
                receivedData.trim();
                parseErrorData(receivedData);
            }
            countEN = countEN + EN;
            
            // EN timeout check
            if (abs(countEN) > 500) {
                if (!enTimeoutActive) {
                    enTimeoutActive = true;
                    EX = 404;
                    EY = 404;
                    mode = 3; // Error mode
                    Serial.println("EN unchanged for 3 seconds. Switching to error mode (3).");
                }
            }
            
            if (EN != prevEN) {
                prevEN = EN;
                enTimeoutActive = false;
                countEN = 0;
                if (mode == 3) {
                    // Reset error state if new EN is received
                    EX = 0;
                    EY = 0;
                    mode = 2; // Default to stopped mode
                    Serial.println("EN changed, clearing timeout error state.");
                }
            }
            
            // Check for 777 auto-shutdown condition
            if (EX == 777 && EY == 777 && mode == 1) {
                // Start countdown if not already started
                if (!isInShutdownCountdown) {
                    isInShutdownCountdown = true;
                    shutdownStartTime = millis();
                    Serial.println("Auto-shutdown sequence started. Will switch to mode 2 in 5 minutes if 777,777 persists.");
                }
                
                // Check if countdown time has elapsed
                if (isInShutdownCountdown && (millis() - shutdownStartTime >= AUTO_SHUTDOWN_TIME)) {
                    Serial.println("Auto-shutdown activated after 5 minutes of 777,777 signal.");
                    mode = 2; // Switch to mode 2 (stopped)
                    setColor(191, 0, 255); // Red light for stopped mode
                    isInShutdownCountdown = false; // Reset shutdown flag
                }
                
                // Blink orange (red+green) during countdown
                if (isInShutdownCountdown) {
                    if (int((millis() - shutdownStartTime)/1000)%2 == 0) {
                        setColor(255, 100, 0); // Orange light
                    } else {
                        setColor(0, 0, 0);
                    }
                }
            } else {
                // Reset shutdown countdown if values change
                if (isInShutdownCountdown) {
                    isInShutdownCountdown = false;
                    Serial.println("Auto-shutdown sequence canceled - 777,777 signal lost.");
                }
            }
            
            // Mode switching logic
            if (millis() - lastCheckTime >= CHECK_INTERVAL) {
                lastCheckTime = millis();    
            
                if (EX == 999) {
                    // Yellow light before starting check
                    // if (stableCount == 0) {
                    //     setColor(255, 255, 0);
                    // }
            
                    // If values remain constant
                    if (EX == prevEX && EY == prevEY) {
                        stableCount++;
            
                        // Blink yellow while waiting for 3 seconds
                        if (stableCount < CHECK_COUNT) {
                            isBlinkOn = !isBlinkOn;
                            if (isBlinkOn) {
                                setColor(0, 255, 255);
                            } else {
                                setColor(0, 0, 0);
                            }
                        }
                    } else {
                        stableCount = 0;
                        setColor(0, 255, 255); // Reset to yellow if values change
                    }
            
                    prevEX = EX;
                    prevEY = EY;
            
                    if (stableCount >= CHECK_COUNT) {
                        mode = EY;
                        if (mode == 1) {
                            setColor(0, 255, 0); // Green light for active mode
                        } else if (mode == 2) {
                            setColor(0, 255, 255); // Red light for stopped mode
                        }
                    }
                } else {
                    stableCount = 0; // Reset if EX is not 999
                    
                    // Don't update LED color here if in auto-shutdown countdown or EN timeout is active
                    if (!isInShutdownCountdown && !enTimeoutActive) {
                        if (mode == 1) {
                            setColor(0, 255, 0); // Green light for active mode
                        } else if (mode == 2) {
                            setColor(0, 0, 255); // Red light for stopped mode
                        } else if (mode == 3) {
                            setColor(255, 0, 0); // Solid red light for error mode
                        }
                    } else if (enTimeoutActive) {
                        // Blinking red for EN timeout error
                        isBlinkOn = !isBlinkOn;
                        if (isBlinkOn) {
                            setColor(255, 0, 0); // Red light
                        } else {
                            setColor(0, 0, 0); // Off
                        }
                    }
                }   
            }
            
            // PID control logic
            if (millis() - lastPIDTime >= PID_INTERVAL && mode == 1) {
                lastPIDTime = millis();
                
                // Only calculate PID if not in special modes
                if (EX != 777 && EX != 999 && EX != 404) {
                    // Calculate PID outputs
                    outputX = calculatePID_X(EX);
                    outputY = calculatePID_Y(EY);
                    
                    // Map PID outputs to motor PWM values
                    int leftMotorPWM = MOTOR_NEUTRAL - outputY - outputX;
                    int rightMotorPWM = MOTOR_NEUTRAL - outputY + outputX;
                    
                    // Constrain PWM values to safe range
                    leftMotorPWM = constrain(leftMotorPWM, 1000, 2000);
                    rightMotorPWM = constrain(rightMotorPWM, 1000, 2000);
                    
                    // Apply motor control
                    motor1.writeMicroseconds(leftMotorPWM);
                    motor2.writeMicroseconds(rightMotorPWM);
                    
                    // Debug output
                    // Serial.print("PID X: ");
                    // Serial.print(outputX);
                    // Serial.print(", PID Y: ");
                    // Serial.print(outputY);
                    // Serial.print(", Left PWM: ");
                    // Serial.print(leftMotorPWM);
                    // Serial.print(", Right PWM: ");
                    // Serial.println(rightMotorPWM);
                }
            }
            
            // Handle special cases and modes
            if (EX == 777 || EX == 999 || EX == 404) {
                // Special codes - stop motors
                motor1.writeMicroseconds(MOTOR_NEUTRAL);
                motor2.writeMicroseconds(MOTOR_NEUTRAL);
            } 
            if (mode == 2 || mode == 3) {
                // Mode 2 or 3 - stopped or error
                digitalWrite(CUTOFF_PIN, LOW);
                motor1.writeMicroseconds(MOTOR_NEUTRAL);
                motor2.writeMicroseconds(MOTOR_NEUTRAL);
                
                // Reset PID variables when stopped
                errorSumX = 0;
                errorSumY = 0;
                lastErrorX = 0;
                lastErrorY = 0;
            } else if (mode == 1) {
                // Mode 1 - active
                digitalWrite(CUTOFF_PIN, HIGH);
                // PID control handled above
            }
            
            // Release mutex
            xSemaphoreGive(dataMutex);
        }
        
        // Short delay to prevent task from consuming too much CPU
        delay(5);
    }
}

// Task for web server and updates
void webServerTask(void *parameter) {
    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/update", HTTP_POST, handleUpdate);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started on Core 1");
    
    for(;;) {
        // Check WiFi connection and reconnect if needed
        if (WiFi.status() != WL_CONNECTED) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                Serial.println("WiFi connection lost. Reconnecting...");
                // setColor(255, 255, 255); // Blue light while reconnecting
                xSemaphoreGive(dataMutex);
            }
            
            WiFi.reconnect();
            
            // Wait for reconnection with timeout
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < maxConnectionAttempts) {
                delay(500);
                Serial.print(".");
                attempts++;
                
                if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                    // Blink blue LED while connecting
                    if (attempts % 2 == 0) {
                        // setColor(255, 255, 255);
                    } else {
                        // setColor(0, 0, 0);
                    }
                    xSemaphoreGive(dataMutex);
                }
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                    Serial.println("");
                    Serial.print("Reconnected to WiFi with IP Address: ");
                    Serial.println(WiFi.localIP());
                    // setColor(0, 255, 255); // Cyan light for successful connection
                    xSemaphoreGive(dataMutex);
                }
                delay(1000);
            } else {
                if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
                    Serial.println("");
                    Serial.println("Failed to reconnect to WiFi.");
                    // setColor(255, 0, 255); // Purple light for connection failure
                    xSemaphoreGive(dataMutex);
                }
                delay(3000);
            }
        }

        // Handle web server clients
        server.handleClient();
        
        // Delay to prevent task from consuming too much CPU
        delay(10);
    }
}

void loadPIDValues() {
    // Read values from EEPROM or use defaults
    EEPROM.get(ADDR_KP_X, Kp_X);
    EEPROM.get(ADDR_KI_X, Ki_X);
    EEPROM.get(ADDR_KD_X, Kd_X);
    EEPROM.get(ADDR_KP_Y, Kp_Y);
    EEPROM.get(ADDR_KI_Y, Ki_Y);
    EEPROM.get(ADDR_KD_Y, Kd_Y);
    
    // Check if values are valid (not NaN or very large)
    if (isnan(Kp_X) || Kp_X < 0 || Kp_X > 100) Kp_X = DEFAULT_KP_X;
    if (isnan(Ki_X) || Ki_X < 0 || Ki_X > 100) Ki_X = DEFAULT_KI_X;
    if (isnan(Kd_X) || Kd_X < 0 || Kd_X > 100) Kd_X = DEFAULT_KD_X;
    if (isnan(Kp_Y) || Kp_Y < 0 || Kp_Y > 100) Kp_Y = DEFAULT_KP_Y;
    if (isnan(Ki_Y) || Ki_Y < 0 || Ki_Y > 100) Ki_Y = DEFAULT_KI_Y;
    if (isnan(Kd_Y) || Kd_Y < 0 || Kd_Y > 100) Kd_Y = DEFAULT_KD_Y;
    
    Serial.println("Loaded PID values:");
    Serial.print("X - Kp: ");
    Serial.print(Kp_X);
    Serial.print(", Ki: ");
    Serial.print(Ki_X);
    Serial.print(", Kd: ");
    Serial.println(Kd_X);
    
    Serial.print("Y - Kp: ");
    Serial.print(Kp_Y);
    Serial.print(", Ki: ");
    Serial.print(Ki_Y);
    Serial.print(", Kd: ");
    Serial.println(Kd_Y);
}

void savePIDValues() {
    EEPROM.put(ADDR_KP_X, Kp_X);
    EEPROM.put(ADDR_KI_X, Ki_X);
    EEPROM.put(ADDR_KD_X, Kd_X);
    EEPROM.put(ADDR_KP_Y, Kp_Y);
    EEPROM.put(ADDR_KI_Y, Ki_Y);
    EEPROM.put(ADDR_KD_Y, Kd_Y);
    EEPROM.commit();
    Serial.println("PID values saved to EEPROM");
}

void handleRoot() {
    // Take mutex before accessing shared data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        String html = "<!DOCTYPE html><html>";
        html += "<head><meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<title>Underwater Robot PID Control</title>";
        html += "<style>";
        html += "body { font-family: Arial; margin: 0; padding: 20px; background-color: #f0f8ff; }";
        html += ".container { max-width: 800px; margin: 0 auto; background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }";
        html += "h1 { color: #00008b; }";
        html += "h2 { color: #4682b4; margin-top: 20px; }";
        html += "table { width: 100%; border-collapse: collapse; margin-bottom: 20px; }";
        html += "th, td { padding: 10px; text-align: left; border-bottom: 1px solid #ddd; }";
        html += "th { background-color: #4682b4; color: white; }";
        html += "input[type='number'] { width: 100%; padding: 8px; box-sizing: border-box; }";
        html += "input[type='submit'] { background-color: #4682b4; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; }";
        html += "input[type='submit']:hover { background-color: #00008b; }";
        html += ".status { margin-top: 20px; padding: 10px; border-radius: 5px; }";
        html += ".current { background-color: #e0f7fa; border-left: 5px solid #4682b4; }";
        html += "</style></head>";
        html += "<body>";
        html += "<div class='container'>";
        html += "<h1>Underwater Robot PID Control</h1>";
        
        html += "<div class='status current'>";
        html += "<h2>Current Status</h2>";
        html += "<p>Device IP: " + WiFi.localIP().toString() + "</p>";
        html += "<p>WiFi Signal: " + String(WiFi.RSSI()) + " dBm</p>";
        html += "<p>Mode: " + String(mode == 1 ? "Active (1)" : (mode == 2 ? "Stopped (2)" : "Error (3)")) + "</p>";
        html += "<p>Last Error X: " + String(EX) + "</p>";
        html += "<p>Last Error Y: " + String(EY) + "</p>";
        
        // Add EN timeout display if active
        if (enTimeoutActive) {
            unsigned long timeElapsed = (millis() - lastENChangeTime) / 1000;
            html += "<p>EN unchanged for: " + String(timeElapsed) + " seconds</p>";
        }
        
        // Add shutdown countdown display if active
        if (isInShutdownCountdown) {
            unsigned long timeRemaining = (AUTO_SHUTDOWN_TIME - (millis() - shutdownStartTime)) / 1000;
            html += "<p>Auto-shutdown in: " + String(timeRemaining) + " seconds</p>";
        }
        
        html += "</div>";
        
        html += "<h2>PID Parameters</h2>";
        html += "<form action='/update' method='POST'>";
        
        html += "<h3>X Axis (Rotation) Parameters</h3>";
        html += "<table>";
        html += "<tr><th>Parameter</th><th>Value</th></tr>";
        html += "<tr><td>Kp:</td><td><input type='number' name='kpx' step='0.01' value='" + String(Kp_X) + "' required></td></tr>";
        html += "<tr><td>Ki:</td><td><input type='number' name='kix' step='0.001' value='" + String(Ki_X) + "' required></td></tr>";
        html += "<tr><td>Kd:</td><td><input type='number' name='kdx' step='0.01' value='" + String(Kd_X) + "' required></td></tr>";
        html += "</table>";
        
        html += "<h3>Y Axis (Forward/Backward) Parameters</h3>";
        html += "<table>";
        html += "<tr><th>Parameter</th><th>Value</th></tr>";
        html += "<tr><td>Kp:</td><td><input type='number' name='kpy' step='0.01' value='" + String(Kp_Y) + "' required></td></tr>";
        html += "<tr><td>Ki:</td><td><input type='number' name='kiy' step='0.001' value='" + String(Ki_Y) + "' required></td></tr>";
        html += "<tr><td>Kd:</td><td><input type='number' name='kdy' step='0.01' value='" + String(Kd_Y) + "' required></td></tr>";
        html += "</table>";
        
        html += "<input type='submit' value='Update Parameters'>";
        html += "</form>";
        
        html += "</div>";
        html += "</body></html>";
        
        server.send(200, "text/html", html);
        
        // Release mutex
        xSemaphoreGive(dataMutex);
    } else {
        // If we can't get the mutex, send a simple response
        server.send(503, "text/plain", "Server busy, try again");
    }
}

void handleUpdate() {
    if (server.hasArg("kpx") && server.hasArg("kix") && server.hasArg("kdx") && 
        server.hasArg("kpy") && server.hasArg("kiy") && server.hasArg("kdy")) {
        
        // Take mutex before updating shared data
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            // Update X axis PID parameters
            Kp_X = server.arg("kpx").toFloat();
            Ki_X = server.arg("kix").toFloat();
            Kd_X = server.arg("kdx").toFloat();
            
            // Update Y axis PID parameters
            Kp_Y = server.arg("kpy").toFloat();
            Ki_Y = server.arg("kiy").toFloat();
            Kd_Y = server.arg("kdy").toFloat();
            
            // Save to EEPROM
            savePIDValues();
            
            // Reset accumulated errors when parameters change
            errorSumX = 0;
            errorSumY = 0;
            
            // Release mutex
            xSemaphoreGive(dataMutex);
            
            server.sendHeader("Location", "/");
            server.send(303);
        } else {
            server.send(503, "text/plain", "Server busy, try again");
        }
    } else {
        server.send(400, "text/plain", "Missing parameters");
    }
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void setColor(int red, int green, int blue) {
    analogWrite(RED_PIN, red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN, blue);
}

// PID controller for X axis
float calculatePID_X(float error) {
    // Calculate P term
    float pTerm = Kp_X * error;
    
    // Calculate I term with anti-windup
    errorSumX += error;
    errorSumX = constrain(errorSumX, -500, 500);  // Prevent excessive integration
    float iTerm = Ki_X * errorSumX;
    
    // Calculate D term
    float dError = error - lastErrorX;
    float dTerm = Kd_X * dError;
    
    // Save error for next iteration
    lastErrorX = error;
    
    // Calculate output
    return pTerm + iTerm + dTerm;
}

// PID controller for Y axis
float calculatePID_Y(float error) {
    // Calculate P term
    float pTerm = Kp_Y * error;
    
    // Calculate I term with anti-windup
    errorSumY += error;
    errorSumY = constrain(errorSumY, -500, 500);  // Prevent excessive integration
    float iTerm = Ki_Y * errorSumY;
    
    // Calculate D term
    float dError = error - lastErrorY;
    float dTerm = Kd_Y * dError;
    
    // Save error for next iteration
    lastErrorY = error;
    
    // Calculate output
    return pTerm + iTerm + dTerm;
}

void parseErrorData(String data) {
    int firstCommaIndex = data.indexOf(',');
    
    if (firstCommaIndex > 0) {
        String exStr = data.substring(0, firstCommaIndex);
        
        int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
        
        if (secondCommaIndex > 0) {
            String eyStr = data.substring(firstCommaIndex + 1, secondCommaIndex);
            String enStr = data.substring(secondCommaIndex + 1);
            
            EX = exStr.toInt();
            EY = eyStr.toInt();
            EN = enStr.toInt();
        } else {
            setColor(255, 255, 255);
        }
    } else {
        setColor(255, 255, 255);
    }
}