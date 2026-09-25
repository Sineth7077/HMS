/**
 * @file micro_climate_nursery.ino
 * @brief ESP32 Automated Commercial Micro-Climate Nursery controller.
 *
 * COMP50069 Scenario 2 - Automated Commercial Micro-Climate Nursery
 *
 * Implements autonomous climate control, manual override,
 * sensor fault safety handling, ADC light sensing,
 * PWM servo control, I2C OLED display and UART monitoring.
 *
 * System priority:
 * 1. SAFETY_ERROR - Sensor failure always has highest priority
 * 2. MANUAL_OVERRIDE - Operator intentionally bypasses automatic control
 * 3. AUTONOMOUS - Normal environmental control
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ============================================================================
// PIN ASSIGNMENTS
// ============================================================================

/** @brief DHT22 temperature/humidity sensor data pin */
constexpr int PIN_DHT22 = 15;

/** @brief LDR photoresistor analog input pin */
constexpr int PIN_LDR = 34;

/** @brief Servo motor PWM control pin */
constexpr int PIN_SERVO = 18;

/** @brief OLED I2C SDA pin */
constexpr int PIN_OLED_SDA = 21;

/** @brief OLED I2C SCL pin */
constexpr int PIN_OLED_SCL = 22;

/** @brief Grow light LED 1 pin */
constexpr int PIN_LED1 = 25;

/** @brief Grow light LED 2 pin */
constexpr int PIN_LED2 = 26;

/** @brief Manual override button pin */
constexpr int PIN_BUTTON = 27;

// ============================================================================
// SENSOR AND ACTUATOR CONFIGURATION
// ============================================================================

/** @brief DHT22 sensor type */
constexpr int DHT_TYPE = DHT22;

/** @brief Servo closed position angle (degrees) */
constexpr int SERVO_CLOSED_ANGLE = 0;

/** @brief Servo open position angle (degrees) */
constexpr int SERVO_OPEN_ANGLE = 90;

/** @brief OLED screen width in pixels */
constexpr int OLED_WIDTH = 128;

/** @brief OLED screen height in pixels */
constexpr int OLED_HEIGHT = 64;

/** @brief OLED I2C address */
constexpr int OLED_ADDRESS = 0x3C;

// ============================================================================
// THRESHOLDS AND LIMITS
// ============================================================================

/** @brief Temperature threshold for proportional servo control (lower bound) */
constexpr float TEMP_CLOSE_THRESHOLD = 28.0;

/** @brief Light level threshold for grow lights (ADC value) */
constexpr int LIGHT_THRESHOLD = 1500;

/** @brief Minimum valid temperature (°C) */
constexpr float TEMP_MIN_VALID = -40.0;

/** @brief Maximum valid temperature (°C) - firmware validation limit */
constexpr float TEMP_MAX_VALID = 75.0;

/** @brief Minimum valid humidity (%) */
constexpr float HUMIDITY_MIN_VALID = 0.0;

/** @brief Maximum valid humidity (%) */
constexpr float HUMIDITY_MAX_VALID = 100.0;

/** @brief Minimum valid LDR ADC value - conservative lower bound to detect disconnected sensor */
constexpr int LDR_MIN_VALID = 100;

/** @brief Maximum valid LDR ADC value - conservative upper bound to detect disconnected sensor */
constexpr int LDR_MAX_VALID = 4000;

// ============================================================================
// TIMING INTERVALS (milliseconds)
// ============================================================================

/** @brief Sensor reading interval */
constexpr unsigned long SENSOR_UPDATE_MS = 500;

/** @brief OLED display update interval */
constexpr unsigned long OLED_UPDATE_MS = 500;

/** @brief Serial output update interval */
constexpr unsigned long SERIAL_UPDATE_MS = 1000;

/** @brief Button debounce interval */
constexpr unsigned long DEBOUNCE_MS = 50;

/** @brief Sensor stability check interval for safety recovery */
constexpr unsigned long SAFETY_STABILITY_MS = 2000;

// ============================================================================
// SYSTEM MODES
// ============================================================================

/**
 * @brief System operational modes
 *
 * MODE_AUTONOMOUS: Normal automatic climate control
 * MODE_MANUAL: Manual override by operator
 * MODE_SAFETY: Safety state due to sensor failure
 */
enum SystemMode {
  MODE_AUTONOMOUS,
  MODE_MANUAL,
  MODE_SAFETY
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Sensor readings
float temperature = 0.0;
float humidity = 0.0;
int lightLevel = 0;

// System state
SystemMode currentMode = MODE_AUTONOMOUS;
int ventAngle = 0;
bool growLightsOn = false;

// Sensor validation
bool dhtValid = true;
bool ldrValid = true;
bool dhtFailure = false;
bool ldrFailure = false;

/** @brief Tracks whether the latest DHT22 reading was successful */
bool dhtLatestReadingValid = false;

// Timing variables
unsigned long lastSensorUpdate = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long sensorValidStartTime = 0;

// Button state
volatile bool buttonPressed = false;
unsigned long lastButtonInterruptTime = 0;

// Hardware timer state
volatile bool timerFlag = false;
volatile unsigned long timerTickCounter = 0;
hw_timer_t* timer = NULL;

// Objects
DHT dht(PIN_DHT22, DHT_TYPE);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Servo ventServo;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupHardware();
void readSensors(unsigned long currentTime);
void validateSensors();
void handleButton(unsigned long currentTime);
void handleUARTCommands(unsigned long currentTime);
void updateSystemState(unsigned long currentTime);
void updateAutonomousControl();
void updateSafetyControl();
void controlOutputs(unsigned long currentTime);
void updateOLED(unsigned long currentTime);
void updateSerial(unsigned long currentTime);
void setVentPosition(int angle);
void setGrowLights(bool on);
int calculateVentAngle(float temp);
bool isDHTDataValid(float temp, float hum);
bool isLDRDataValid(int light);
String getModeString(SystemMode mode);
String getVentStatus();
void printModeChange(SystemMode oldMode, SystemMode newMode);
void printSafetyAlert(const char* sensor, const char* message);
void printRecovery();
void printUARTCommand(const char* cmd, bool accepted);
void printHelp();
void IRAM_ATTR buttonISR();
void IRAM_ATTR timerISR();
void setupHardwareTimer();
void handleTimer();

// ============================================================================
// SETUP
// ============================================================================

/**
 * @brief Initialize hardware and system components
 * @return void
 */
void setup() {
  Serial.begin(115200);
  
  setupHardware();
  
  Serial.println(F("========================================"));
  Serial.println(F("Micro-Climate Nursery Controller"));
  Serial.println(F("COMP50069 Scenario 2"));
  Serial.println(F("========================================"));
  Serial.println(F("System Initialized"));
  Serial.println(F("Mode: AUTONOMOUS"));
  Serial.println(F("========================================"));
}

/**
 * @brief Configure all hardware peripherals
 * @return void
 */
void setupHardware() {
  // Initialize DHT22 sensor
  dht.begin();
  
  // Initialize OLED display
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED initialization failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Initialize servo
  ventServo.attach(PIN_SERVO);
  setVentPosition(false);
  
  // Initialize LED pins
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  setGrowLights(false);
  
  // Initialize button with internal pullup and interrupt
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), buttonISR, FALLING);
  
  // Initialize hardware timer
  setupHardwareTimer();
  
  // Initial sensor read will occur on first sensor update interval
  // No blocking delay - system remains non-blocking
}

// ============================================================================
// MAIN LOOP
// ============================================================================

/**
 * @brief Main program loop - non-blocking implementation
 * @return void
 */
void loop() {
  unsigned long now = millis();
  
  handleTimer();
  readSensors(now);
  handleButton(now);
  handleUARTCommands(now);
  validateSensors();
  updateSystemState(now);
  controlOutputs(now);
  updateOLED(now);
  updateSerial(now);
}

// ============================================================================
// SENSOR READING
// ============================================================================

/**
 * @brief Read all sensor values with non-blocking timing
 * @param currentTime Current system time in milliseconds
 * @return void
 */
void readSensors(unsigned long currentTime) {
  if (currentTime - lastSensorUpdate < SENSOR_UPDATE_MS) {
    return;
  }
  lastSensorUpdate = currentTime;
  
  // Read DHT22
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();
  
  // Track whether the latest reading was successful
  dhtLatestReadingValid = (!isnan(newTemp) && !isnan(newHum));
  
  if (dhtLatestReadingValid) {
    temperature = newTemp;
    humidity = newHum;
  }
  
  // Read LDR
  lightLevel = analogRead(PIN_LDR);
}

// ============================================================================
// SENSOR VALIDATION
// ============================================================================

/**
 * @brief Validate all sensor readings and detect failures
 * @return void
 */
void validateSensors() {
  // DHT22 validation: check if latest reading was successful AND values are in valid range
  dhtValid = dhtLatestReadingValid && isDHTDataValid(temperature, humidity);
  ldrValid = isLDRDataValid(lightLevel);
  
  dhtFailure = !dhtValid;
  ldrFailure = !ldrValid;
}

/**
 * @brief Check if DHT22 data is within valid range
 * @param temp Temperature value in Celsius
 * @param hum Humidity value in percentage
 * @return true if data is valid, false otherwise
 */
bool isDHTDataValid(float temp, float hum) {
  if (isnan(temp) || isnan(hum)) {
    return false;
  }
  if (temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID) {
    return false;
  }
  if (hum < HUMIDITY_MIN_VALID || hum > HUMIDITY_MAX_VALID) {
    return false;
  }
  return true;
}

/**
 * @brief Check if LDR data is within valid range
 * Uses conservative bounds to detect disconnected or faulty sensors
 * @param light ADC light level reading (0-4095)
 * @return true if data is valid, false otherwise
 */
bool isLDRDataValid(int light) {
  if (light < LDR_MIN_VALID || light > LDR_MAX_VALID) {
    return false;
  }
  return true;
}

// ============================================================================
// UART COMMAND HANDLING
// ============================================================================

/**
 * @brief Handle UART command input with non-blocking processing
 * Processes commands: A, M, S, H
 * UART commands cannot bypass safety mode
 * @param currentTime Current system time in milliseconds (unused, for interface consistency)
 * @return void
 */
void handleUARTCommands(unsigned long currentTime) {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Wait for newline to process full command
    if (cmd == '\n' || cmd == '\r') {
      continue;
    }
    
    // Simple single-character commands
    switch (toupper(cmd)) {
      case 'A':
        // Request Autonomous mode
        if (dhtFailure || ldrFailure) {
          printUARTCommand("A", false);
          Serial.println(F("Rejected - Sensor fault active"));
        } else if (currentMode == MODE_SAFETY) {
          printUARTCommand("A", false);
          Serial.println(F("Rejected - System in SAFETY mode"));
        } else {
          printUARTCommand("A", true);
          if (currentMode != MODE_AUTONOMOUS) {
            printModeChange(currentMode, MODE_AUTONOMOUS);
            currentMode = MODE_AUTONOMOUS;
          }
        }
        break;
        
      case 'M':
        // Request Manual mode
        if (dhtFailure || ldrFailure) {
          printUARTCommand("M", false);
          Serial.println(F("Rejected - Sensor fault active"));
        } else if (currentMode == MODE_SAFETY) {
          printUARTCommand("M", false);
          Serial.println(F("Rejected - System in SAFETY mode"));
        } else {
          printUARTCommand("M", true);
          if (currentMode != MODE_MANUAL) {
            printModeChange(currentMode, MODE_MANUAL);
            currentMode = MODE_MANUAL;
          }
        }
        break;
        
      case 'S':
        // Show Safety status
        printUARTCommand("S", true);
        Serial.println(F("Note: Safety mode is automatically entered on sensor fault"));
        Serial.println(F("Current sensor status:"));
        Serial.print(F("  DHT22: "));
        Serial.println(dhtValid ? F("VALID") : F("INVALID"));
        Serial.print(F("  LDR: "));
        Serial.println(ldrValid ? F("VALID") : F("INVALID"));
        break;
        
      case 'H':
        // Help command
        printHelp();
        break;
        
      default:
        // Unknown command
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
        Serial.println(F("Type 'H' for help"));
        break;
    }
  }
}

/**
 * @brief Print UART command received and acceptance status
 * @param cmd Command character received
 * @param accepted Whether the command was accepted
 * @return void
 */
void printUARTCommand(const char* cmd, bool accepted) {
  Serial.print(F("[UART] Command: "));
  Serial.print(cmd);
  Serial.print(F(" - "));
  Serial.println(accepted ? F("ACCEPTED") : F("REJECTED"));
}

/**
 * @brief Print available UART commands
 * @return void
 */
void printHelp() {
  Serial.println(F("========================================"));
  Serial.println(F("Available Commands:"));
  Serial.println(F("  A - Request Autonomous mode"));
  Serial.println(F("  M - Request Manual mode"));
  Serial.println(F("  S - Show Safety status"));
  Serial.println(F("  H - Print this help"));
  Serial.println(F("========================================"));
  Serial.println(F("Note: Commands cannot bypass Safety mode"));
  Serial.println(F("========================================"));
}

// ============================================================================
// HARDWARE TIMER
// ============================================================================

/**
 * @brief Hardware timer interrupt service routine
 * Sets flag for main loop to process timer event
 * ISR must be extremely short - only sets a flag
 * @return void
 */
void IRAM_ATTR timerISR() {
  timerFlag = true;
}

/**
 * @brief Initialize hardware timer for periodic interrupt
 * Configures ESP32 hardware timer to trigger interrupt every 1 second
 * Provides system tick counter for interrupt-driven periodic scheduling
 * Uses ESP32 Arduino core 3.x timer API
 * @return void
 */
void setupHardwareTimer() {
  // Timer with 1 MHz frequency (ESP32 Arduino core 3.x API)
  timer = timerBegin(1000000);
  
  // Attach interrupt handler
  timerAttachInterrupt(timer, &timerISR);
  
  // Set alarm to trigger every 1,000,000 counts (1 second at 1 MHz)
  // Arguments: timer, alarm_value, autoreload, reload_count (0 = infinite)
  timerAlarm(timer, 1000000, true, 0);
}

/**
 * @brief Handle timer interrupt flag in main loop
 * Processes timer events safely outside of ISR context
 * Increments system tick counter for periodic scheduling
 * @return void
 */
void handleTimer() {
  if (timerFlag) {
    timerFlag = false;
    timerTickCounter++;
    // Hardware timer provides periodic 1-second system tick
    // This can be used for time-based events independent of millis()
  }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

/**
 * @brief Button interrupt service routine
 * Sets flag for main loop to process button press
 * ISR must be extremely short - only sets a flag
 * @return void
 */
void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

/**
 * @brief Handle manual override button with non-blocking debounce
 * Toggles between AUTONOMOUS and MANUAL modes on button press
 * Cannot exit SAFETY mode via button
 * @param currentTime Current system time in milliseconds
 * @return void
 */
void handleButton(unsigned long currentTime) {
  if (!buttonPressed) {
    return;
  }
  
  // Debounce check
  if (currentTime - lastButtonInterruptTime < DEBOUNCE_MS) {
    return;
  }
  
  buttonPressed = false;
  lastButtonInterruptTime = currentTime;
  
  // Safety mode has highest priority - button cannot bypass it
  if (currentMode == MODE_SAFETY) {
    Serial.println(F("[BUTTON] Ignored - System in SAFETY mode"));
    return;
  }
  
  SystemMode newMode;
  
  if (currentMode == MODE_AUTONOMOUS) {
    newMode = MODE_MANUAL;
  } else if (currentMode == MODE_MANUAL) {
    newMode = MODE_AUTONOMOUS;
  } else {
    return;
  }
  
  printModeChange(currentMode, newMode);
  currentMode = newMode;
}

// ============================================================================
// SYSTEM STATE MANAGEMENT
// ============================================================================

/**
 * @brief Update system state based on sensor validation and mode
 * Priority: SAFETY > MANUAL > AUTONOMOUS
 * SAFETY mode has highest priority, overrides all other modes
 * @param currentTime Current system time in milliseconds
 * @return void
 */
void updateSystemState(unsigned long currentTime) {
  
  // Priority 1: SAFETY - Sensor failure always has highest priority
  if (dhtFailure || ldrFailure) {
    if (currentMode != MODE_SAFETY) {
      if (dhtFailure) {
        printSafetyAlert("DHT22", "SENSOR FAILURE");
      } else if (ldrFailure) {
        printSafetyAlert("LDR", "SENSOR FAILURE");
      }
      currentMode = MODE_SAFETY;
    }
    // ALWAYS reset recovery timer on ANY sensor fault
    // This cancels recovery if fault occurs during recovery period
    sensorValidStartTime = 0;
  } else {
    // Sensors are valid
    if (currentMode == MODE_SAFETY) {
      // Start recovery timer on first valid reading
      if (sensorValidStartTime == 0) {
        sensorValidStartTime = currentTime;
      }
      
      // Require continuous valid readings for recovery period (>= 2000ms)
      if (currentTime - sensorValidStartTime >= SAFETY_STABILITY_MS) {
        printRecovery();
        currentMode = MODE_AUTONOMOUS;
        sensorValidStartTime = 0;
      }
      // If fault occurs during recovery, sensorValidStartTime will be reset above
    } else {
      // Not in safety mode, ensure recovery timer is reset
      sensorValidStartTime = 0;
    }
  }
  
  // Priority 2: MANUAL - Only when sensors valid and not in safety
  // Priority 3: AUTONOMOUS - Normal operation
  // Apply mode-specific control logic
  if (currentMode == MODE_AUTONOMOUS) {
    updateAutonomousControl();
  } else if (currentMode == MODE_MANUAL) {
    // Manual override: force vent open (90°), disable automatic control
    ventAngle = SERVO_OPEN_ANGLE; // Synchronize ventAngle with physical position
    // In manual mode, grow lights are also manually controlled
    // Safe default: turn on grow lights in manual mode
    growLightsOn = true;
  } else if (currentMode == MODE_SAFETY) {
    updateSafetyControl();
  }
}

/**
 * @brief Update autonomous control logic based on sensors
 * Implements calculated servo position based on temperature using proportional control
 * Implements light threshold control for grow lights
 * @return void
 */
void updateAutonomousControl() {
  // Calculate servo position based on temperature (proportional control)
  ventAngle = calculateVentAngle(temperature);
  
  // Light control
  growLightsOn = (lightLevel < LIGHT_THRESHOLD);
}

/**
 * @brief Update safety control logic
 * Places system in safe physical posture: vent open (90°), grow lights on
 * Synchronizes ventAngle with physical servo position
 * @return void
 */
void updateSafetyControl() {
  // Safe posture: vent open to prevent overheating
  ventAngle = SERVO_OPEN_ANGLE; // Synchronize ventAngle with physical position
  
  // In safety mode, turn on grow lights as a safe default
  growLightsOn = true;
}

// ============================================================================
// OUTPUT CONTROL
// ============================================================================

/**
 * @brief Control all physical outputs based on current system state
 * @param currentTime Current system time in milliseconds (unused, for interface consistency)
 * @return void
 */
void controlOutputs(unsigned long currentTime) {
  // In MANUAL or SAFETY modes, force vent to open position
  if (currentMode == MODE_MANUAL || currentMode == MODE_SAFETY) {
    setVentPosition(SERVO_OPEN_ANGLE);
  } else {
    // In AUTONOMOUS mode, use calculated angle
    setVentPosition(ventAngle);
  }
  setGrowLights(growLightsOn);
}

/**
 * @brief Calculate servo vent angle based on temperature using floating-point interpolation
 * Maps temperature to servo angle: <=28°C → 0°, 28-32°C → proportional, >=32°C → 90°
 * Uses floating-point linear interpolation for proportional control
 * Clamps result between SERVO_CLOSED_ANGLE and SERVO_OPEN_ANGLE
 * @param temp Current temperature in Celsius
 * @return Calculated servo angle in degrees (0-90)
 */
int calculateVentAngle(float temp) {
  float angle;
  
  // Below 28°C: 0° (closed)
  if (temp <= TEMP_CLOSE_THRESHOLD) {
    angle = SERVO_CLOSED_ANGLE;
  }
  // Above 32°C: 90° (fully open)
  else if (temp >= 32.0) {
    angle = SERVO_OPEN_ANGLE;
  }
  // Between 28°C and 32°C: proportional interpolation
  else {
    // Linear interpolation: angle = (temp - 28) / (32 - 28) * 90
    float normalized = (temp - TEMP_CLOSE_THRESHOLD) / (32.0 - TEMP_CLOSE_THRESHOLD);
    angle = normalized * SERVO_OPEN_ANGLE;
  }
  
  // Clamp to valid servo range
  if (angle < SERVO_CLOSED_ANGLE) angle = SERVO_CLOSED_ANGLE;
  if (angle > SERVO_OPEN_ANGLE) angle = SERVO_OPEN_ANGLE;
  
  return (int)angle;
}

/**
 * @brief Set servo vent position to specific angle
 * @param angle Servo angle in degrees (0-90)
 * @return void
 */
void setVentPosition(int angle) {
  ventServo.write(angle);
}

/**
 * @brief Control grow light LEDs (both LEDs together)
 * @param on True to turn on both grow lights, false to turn off
 * @return void
 */
void setGrowLights(bool on) {
  digitalWrite(PIN_LED1, on ? HIGH : LOW);
  digitalWrite(PIN_LED2, on ? HIGH : LOW);
}

// ============================================================================
// OLED DISPLAY
// ============================================================================

/**
 * @brief Update OLED display with non-blocking timing
 * Display content varies by system mode (AUTO/MANUAL/SAFETY)
 * @param currentTime Current system time in milliseconds
 * @return void
 */
void updateOLED(unsigned long currentTime) {
  if (currentTime - lastOLEDUpdate < OLED_UPDATE_MS) {
    return;
  }
  lastOLEDUpdate = currentTime;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  if (currentMode == MODE_SAFETY) {
    // Safety mode display
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("!!! SENSOR FAULT !!!"));
    
    display.setCursor(0, 16);
    if (dhtFailure) {
      display.println(F("DHT22 FAILURE"));
    } else if (ldrFailure) {
      display.println(F("LDR FAILURE"));
    }
    
    display.setCursor(0, 32);
    display.print(F("Vent: "));
    display.println(getVentStatus());
    
    display.setCursor(0, 48);
    display.println(F("Check Sensor"));
    
  } else if (currentMode == MODE_MANUAL) {
    // Manual override display
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("MANUAL OVERRIDE"));
    
    display.setCursor(0, 16);
    display.print(F("Vent: "));
    display.println(getVentStatus());
    
    display.setCursor(0, 32);
    display.println(F("Automation: OFF"));
    
    display.setCursor(0, 48);
    display.print(F("Light: "));
    display.println(lightLevel);
    
  } else {
    // Autonomous mode display
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("AUTO MODE"));
    
    display.setCursor(0, 16);
    display.print(F("T:"));
    display.print(temperature, 1);
    display.print(F("C H:"));
    display.print(humidity, 1);
    display.println(F("%"));
    
    display.setCursor(0, 32);
    display.print(F("Light:"));
    display.println(lightLevel);
    
    display.setCursor(0, 48);
    display.print(F("Vent:"));
    display.println(getVentStatus());
  }
  
  display.display();
}

// ============================================================================
// SERIAL OUTPUT
// ============================================================================

/**
 * @brief Update Serial output with non-blocking timing
 * Outputs system status, sensor readings, and actuator states in demonstration format
 * @param currentTime Current system time in milliseconds
 * @return void
 */
void updateSerial(unsigned long currentTime) {
  if (currentTime - lastSerialUpdate < SERIAL_UPDATE_MS) {
    return;
  }
  lastSerialUpdate = currentTime;
  
  if (currentMode == MODE_SAFETY) {
    // Safety mode output format
    Serial.println(F("SAFETY ERROR"));
    if (dhtFailure) {
      Serial.print(F("DHT22 temperature invalid: "));
      Serial.print(temperature, 1);
      Serial.println(F(" C"));
    } else if (ldrFailure) {
      Serial.print(F("LDR invalid: "));
      Serial.println(lightLevel);
    }
    Serial.print(F("Vent: SAFE OPEN ("));
    Serial.print(ventAngle);
    Serial.println(F(" deg)"));
    Serial.print(F("Grow Lights: "));
    Serial.println(growLightsOn ? F("ON") : F("OFF"));
    Serial.print(F("Recovery: "));
    if (sensorValidStartTime > 0) {
      unsigned long elapsed = millis() - sensorValidStartTime;
      Serial.print(elapsed / 1000);
      Serial.println(F("s / 2s"));
    } else {
      Serial.println(F("Waiting for valid sensor data"));
    }
  } else {
    // Normal mode output format
    Serial.print(F("MODE: "));
    Serial.println(getModeString(currentMode));
    Serial.print(F("Temperature: "));
    Serial.print(temperature, 1);
    Serial.println(F(" C"));
    Serial.print(F("Humidity: "));
    Serial.print(humidity, 1);
    Serial.println(F(" %"));
    Serial.print(F("Light: "));
    Serial.println(lightLevel);
    Serial.print(F("Vent Angle: "));
    Serial.print(ventAngle);
    Serial.println(F(" deg"));
    Serial.print(F("Grow Lights: "));
    Serial.println(growLightsOn ? F("ON") : F("OFF"));
    Serial.print(F("Sensor: "));
    Serial.println((dhtValid && ldrValid) ? F("VALID") : F("INVALID"));
  }
  Serial.println(F("----------------------------------------"));
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Get string representation of system mode
 * @param mode System mode enum value (MODE_AUTONOMOUS, MODE_MANUAL, MODE_SAFETY)
 * @return String representation of mode for display/logging
 */
String getModeString(SystemMode mode) {
  switch (mode) {
    case MODE_AUTONOMOUS:
      return F("AUTONOMOUS");
    case MODE_MANUAL:
      return F("MANUAL OVERRIDE");
    case MODE_SAFETY:
      return F("SAFETY ERROR");
    default:
      return F("UNKNOWN");
  }
}

/**
 * @brief Get vent status string for OLED display
 * Returns "CLOSED" when vent angle is 0°, "OPEN" when vent angle > 0°
 * @return Vent status string
 */
String getVentStatus() {
  if (ventAngle == 0) {
    return F("CLOSED");
  } else {
    return F("OPEN");
  }
}

/**
 * @brief Print mode change message to Serial
 * Logs mode transitions for debugging and monitoring
 * @param oldMode Previous system mode before change
 * @param newMode New system mode after change
 * @return void
 */
void printModeChange(SystemMode oldMode, SystemMode newMode) {
  Serial.println(F("[MODE CHANGE]"));
  Serial.print(getModeString(oldMode));
  Serial.print(F(" -> "));
  Serial.println(getModeString(newMode));
}

/**
 * @brief Print safety alert message to Serial
 * Logs sensor failure and transition to safety mode
 * @param sensor Name of failed sensor (e.g., "DHT22", "LDR")
 * @param message Additional information about the failure
 * @return void
 */
void printSafetyAlert(const char* sensor, const char* message) {
  Serial.println(F("[SAFETY]"));
  Serial.print(sensor);
  Serial.print(F(" "));
  Serial.println(message);
  Serial.println(F("Switching to SAFE STATE"));
  Serial.println(F("Vent -> OPEN"));
}

/**
 * @brief Print recovery message to Serial
 * Logs successful sensor recovery and return to autonomous mode
 * @return void
 */
void printRecovery() {
  Serial.println(F("[RECOVERY]"));
  Serial.println(F("Sensors valid"));
  Serial.println(F("Leaving SAFETY ERROR"));
  Serial.println(F("Returning to AUTONOMOUS"));
}
