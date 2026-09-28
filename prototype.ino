/*
 * COMP50069 - Hardware, Microcontrollers and Sensors
 * Scenario 2 - Automated Commercial Micro-Climate Nursery
 *
 * ESP32 Automated Nursery Controller
 *
 * Hardware:
 * - ESP32
 * - DHT22 Temperature/Humidity Sensor
 * - Digital LDR Module
 * - SSD1306 OLED Display
 * - SG90 9g Servo Motor
 * - WS2812B Addressable LED Strip (44 LEDs)
 * - Push Button
 *
 * Servo Vent Positions:
 * - 0 degrees  = CLOSED
 * - 90 degrees = OPEN
 *
 * Automatic Temperature Control:
 * - <= 28°C = CLOSED
 * - >= 32°C = OPEN
 * - 29-31°C = CLOSED
 *
 * Modes:
 * - A = Autonomous
 * - M = Manual
 * - S = Safety
 * - H = Help
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <FastLED.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// DHT22
#define DHT_PIN 15
#define DHT_TYPE DHT22

// LDR Digital Output
#define LDR_PIN 34

// Servo
#define SERVO_PIN 18

// OLED I2C
#define OLED_SDA 21
#define OLED_SCL 22

// Manual Button
#define BUTTON_PIN 27

// WS2812B LED Strip
#define LED_PIN 5
#define NUM_LEDS 44

// ============================================================
// OLED SETTINGS
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// ============================================================
// SENSOR OBJECTS
// ============================================================

DHT dht(DHT_PIN, DHT_TYPE);
Servo ventServo;

CRGB leds[NUM_LEDS];

// ============================================================
// SERVO SETTINGS
// ============================================================

// IMPORTANT:
// Physical prototype calibration:
// 0°  = CLOSED
// 90° = OPEN

const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE   = 90;

// ============================================================
// TEMPERATURE SETTINGS
// ============================================================

const float TEMP_CLOSE_THRESHOLD = 28.0;
const float TEMP_OPEN_THRESHOLD  = 32.0;

// ============================================================
// LED SETTINGS
// ============================================================

const int LED_BRIGHTNESS = 60;

// Purple grow-light colour
const CRGB GROW_LIGHT_COLOUR = CRGB(180, 0, 255);

// ============================================================
// TIMING SETTINGS
// ============================================================

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long DISPLAY_INTERVAL = 500;
const unsigned long SERIAL_INTERVAL = 1000;
const unsigned long BUTTON_DEBOUNCE = 50;
const unsigned long SAFETY_STABLE_TIME = 2000;

// ============================================================
// SYSTEM MODES
// ============================================================

enum SystemMode
{
  AUTONOMOUS,
  MANUAL,
  SAFETY
};

SystemMode currentMode = AUTONOMOUS;

// ============================================================
// SENSOR VARIABLES
// ============================================================

float temperature = 0.0;
float humidity = 0.0;

int lightLevel = LOW;

bool sensorError = false;

// ============================================================
// ACTUATOR VARIABLES
// ============================================================

int ventAngle = SERVO_CLOSED_ANGLE;

bool growLightsOn = false;

// ============================================================
// BUTTON VARIABLES
// ============================================================

volatile bool buttonPressed = false;

unsigned long lastButtonTime = 0;

// ============================================================
// TIMER VARIABLES
// ============================================================

hw_timer_t *timer = NULL;

volatile bool timerFlag = false;

// ============================================================
// TIMER INTERRUPT
// ============================================================

void IRAM_ATTR timerISR()
{
  timerFlag = true;
}

// ============================================================
// BUTTON INTERRUPT
// ============================================================

void IRAM_ATTR buttonISR()
{
  buttonPressed = true;
}

// ============================================================
// SET VENT POSITION
// ============================================================

void setVentAngle(int angle)
{
  // Only allow the two required positions.
  if (angle != SERVO_CLOSED_ANGLE &&
      angle != SERVO_OPEN_ANGLE)
  {
    angle = SERVO_CLOSED_ANGLE;
  }

  ventAngle = angle;

  ventServo.write(ventAngle);
}

// ============================================================
// GET VENT STATUS
// ============================================================

String getVentStatus()
{
  if (ventAngle == SERVO_CLOSED_ANGLE)
  {
    return "CLOSED";
  }

  return "OPEN";
}

// ============================================================
// CALCULATE AUTOMATIC VENT POSITION
// ============================================================

int calculateVentAngle(float temp)
{
  /*
   * Automatic control:
   *
   * <= 28°C  -> 0°  -> CLOSED
   * 29-31°C  -> 0°  -> CLOSED
   * >= 32°C  -> 90° -> OPEN
   *
   * No half-open position.
   */

  if (temp >= TEMP_OPEN_THRESHOLD)
  {
    return SERVO_OPEN_ANGLE;
  }
  else
  {
    return SERVO_CLOSED_ANGLE;
  }
}

// ============================================================
// SET GROW LIGHTS
// ============================================================

void setGrowLights(bool state)
{
  growLightsOn = state;

  if (state)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = GROW_LIGHT_COLOUR;
    }

    FastLED.setBrightness(LED_BRIGHTNESS);
  }
  else
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
  }

  FastLED.show();
}

// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  float newTemperature = dht.readTemperature();
  float newHumidity = dht.readHumidity();

  // Check DHT22 readings
  if (isnan(newTemperature) || isnan(newHumidity))
  {
    sensorError = true;
  }
  else
  {
    temperature = newTemperature;
    humidity = newHumidity;
    sensorError = false;
  }

  // Read digital LDR
  lightLevel = digitalRead(LDR_PIN);
}

// ============================================================
// CHECK IF IT IS DARK
// ============================================================

bool isDark()
{
  /*
   * Digital LDR module used in this project:
   *
   * HIGH = DARK
   * LOW  = BRIGHT
   */

  return lightLevel == HIGH;
}

// ============================================================
// AUTONOMOUS MODE
// ============================================================

void runAutonomousMode()
{
  if (sensorError)
  {
    // Fail-safe behaviour
    setVentAngle(SERVO_OPEN_ANGLE);
    setGrowLights(true);

    return;
  }

  // ----------------------------------------------------------
  // TEMPERATURE CONTROL
  // ----------------------------------------------------------

  int requiredVentAngle = calculateVentAngle(temperature);

  setVentAngle(requiredVentAngle);

  // ----------------------------------------------------------
  // LIGHT CONTROL
  // ----------------------------------------------------------

  if (isDark())
  {
    setGrowLights(true);
  }
  else
  {
    setGrowLights(false);
  }
}

// ============================================================
// MANUAL MODE
// ============================================================

void runManualMode()
{
  /*
   * Manual override:
   *
   * Vent = OPEN
   * Grow lights = ON
   */

  setVentAngle(SERVO_OPEN_ANGLE);

  setGrowLights(true);
}

// ============================================================
// SAFETY MODE
// ============================================================

void runSafetyMode()
{
  /*
   * Safety mode:
   *
   * Vent = OPEN
   * Grow lights = ON
   *
   * This provides maximum ventilation.
   */

  setVentAngle(SERVO_OPEN_ANGLE);

  setGrowLights(true);
}

// ============================================================
// CHANGE SYSTEM MODE
// ============================================================

void changeMode(SystemMode newMode)
{
  currentMode = newMode;

  switch (currentMode)
  {
    case AUTONOMOUS:
      Serial.println("Mode changed to AUTONOMOUS");
      break;

    case MANUAL:
      Serial.println("Mode changed to MANUAL");
      break;

    case SAFETY:
      Serial.println("Mode changed to SAFETY");
      break;
  }
}

// ============================================================
// BUTTON HANDLER
// ============================================================

void handleButton()
{
  if (!buttonPressed)
  {
    return;
  }

  buttonPressed = false;

  unsigned long currentTime = millis();

  if (currentTime - lastButtonTime < BUTTON_DEBOUNCE)
  {
    return;
  }

  lastButtonTime = currentTime;

  /*
   * Button cycles:
   *
   * AUTONOMOUS -> MANUAL -> AUTONOMOUS
   *
   * Safety mode is controlled separately.
   */

  if (currentMode == AUTONOMOUS)
  {
    changeMode(MANUAL);
  }
  else if (currentMode == MANUAL)
  {
    changeMode(AUTONOMOUS);
  }
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void printHelp()
{
  Serial.println();
  Serial.println("=================================");
  Serial.println("AUTOMATED NURSERY COMMANDS");
  Serial.println("=================================");
  Serial.println("A - Autonomous Mode");
  Serial.println("M - Manual Mode");
  Serial.println("S - Safety Mode");
  Serial.println("H - Show Help");
  Serial.println("=================================");
  Serial.println();
}

void handleSerialCommand()
{
  if (!Serial.available())
  {
    return;
  }

  char command = Serial.read();

  command = toupper(command);

  switch (command)
  {
    case 'A':
      changeMode(AUTONOMOUS);
      break;

    case 'M':
      changeMode(MANUAL);
      break;

    case 'S':
      changeMode(SAFETY);
      break;

    case 'H':
      printHelp();
      break;

    default:
      break;
  }
}

// ============================================================
// GET MODE NAME
// ============================================================

String getModeName()
{
  switch (currentMode)
  {
    case AUTONOMOUS:
      return "AUTONOMOUS";

    case MANUAL:
      return "MANUAL";

    case SAFETY:
      return "SAFETY";
  }

  return "UNKNOWN";
}

// ============================================================
// OLED DISPLAY - AUTONOMOUS
// ============================================================

void displayAutonomous()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("AUTO MODE");

  display.setCursor(0, 16);
  display.print(F("T: "));
  display.print(temperature, 1);

  display.print(F("C H:"));

  display.print(humidity, 1);
  display.println("%");

  display.setCursor(0, 32);
  display.print("S.Light: ");

  if (isDark())
  {
    display.println("DARK");
  }
  else
  {
    display.println("BRIGHT");
  }

  display.setCursor(0, 48);
  display.print("Vent: ");

  /*
   * In autonomous mode, display the vent status
   * using the same temperature rule that controls
   * the physical servo.
   *
   * >= 32°C -> OPEN
   * < 32°C  -> CLOSED
   *
   * This is display logic only.
   * It does not change the servo position.
   */

  if (sensorError)
  {
    display.println("OPEN");
  }
  else if (temperature >= TEMP_OPEN_THRESHOLD)
  {
    display.println("OPEN");
  }
  else
  {
    display.println("CLOSED");
  }

  display.display();
}

// ============================================================
// OLED DISPLAY - MANUAL
// ============================================================

void displayManual()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("MANUAL MODE");

  display.setCursor(0, 15);
  display.println("Manual Override");

  display.setCursor(0, 30);
  display.print("Vent: ");
  display.println(getVentStatus());

  display.setCursor(0, 43);
  display.println("Automation: OFF");

  display.setCursor(0, 55);

  if (growLightsOn)
  {
    display.print("Lights: ON");
  }
  else
  {
    display.print("Lights: OFF");
  }

  display.display();
}

// ============================================================
// OLED DISPLAY - SAFETY
// ============================================================

void displaySafety()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("SAFETY MODE");

  display.setCursor(0, 15);
  display.println("Sensor Fault!");

  display.setCursor(0, 30);
  display.print("Vent: ");
  display.println(getVentStatus());

  display.setCursor(0, 43);
  display.println("Ventilation OPEN");

  display.setCursor(0, 55);
  display.println("Lights: ON");

  display.display();
}

// ============================================================
// UPDATE OLED
// ============================================================

void updateDisplay()
{
  switch (currentMode)
  {
    case AUTONOMOUS:
      displayAutonomous();
      break;

    case MANUAL:
      displayManual();
      break;

    case SAFETY:
      displaySafety();
      break;
  }
}

// ============================================================
// SERIAL STATUS
// ============================================================

void printSystemStatus()
{
  Serial.println();
  Serial.println("=================================");
  Serial.println("NURSERY SYSTEM STATUS");
  Serial.println("=================================");

  Serial.print("Mode: ");
  Serial.println(getModeName());

  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");

  Serial.print("Light: ");

  if (isDark())
  {
    Serial.println("DARK");
  }
  else
  {
    Serial.println("BRIGHT");
  }

  Serial.print("Vent Angle: ");
  Serial.print(ventAngle);
  Serial.println(" degrees");

  Serial.print("Vent Status: ");
  Serial.println(getVentStatus());

  Serial.print("Grow Lights: ");

  if (growLightsOn)
  {
    Serial.println("ON");
  }
  else
  {
    Serial.println("OFF");
  }

  Serial.print("LED Count: ");
  Serial.println(NUM_LEDS);

  Serial.print("Sensor Status: ");

  if (sensorError)
  {
    Serial.println("ERROR");
  }
  else
  {
    Serial.println("OK");
  }

  Serial.println("=================================");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("AUTOMATED COMMERCIAL");
  Serial.println("MICRO-CLIMATE NURSERY");
  Serial.println("=================================");

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(OLED_SDA, OLED_SCL);

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS))
  {
    Serial.println("OLED initialization failed!");
  }
  else
  {
    Serial.println("OLED initialized.");
  }

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Automated Nursery");

  display.setCursor(0, 15);
  display.println("System Starting...");

  display.display();

  // ----------------------------------------------------------
  // DHT22
  // ----------------------------------------------------------

  dht.begin();

  Serial.println("DHT22 initialized.");

  // ----------------------------------------------------------
  // LDR
  // ----------------------------------------------------------

  pinMode(LDR_PIN, INPUT);

  Serial.println("LDR initialized.");

  // ----------------------------------------------------------
  // BUTTON
  // ----------------------------------------------------------

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    buttonISR,
    FALLING
  );

  Serial.println("Button initialized.");

  // ----------------------------------------------------------
  // SERVO
  // ----------------------------------------------------------

  ventServo.setPeriodHertz(50);

  ventServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  // Start with vent CLOSED
  setVentAngle(SERVO_CLOSED_ANGLE);

  Serial.println("Servo initialized.");
  Serial.println("0 degrees = CLOSED");
  Serial.println("90 degrees = OPEN");

  // ----------------------------------------------------------
  // LED STRIP
  // ----------------------------------------------------------

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  FastLED.setBrightness(LED_BRIGHTNESS);

  setGrowLights(false);

  Serial.println("LED strip initialized.");

  // ----------------------------------------------------------
  // HARDWARE TIMER
  // ----------------------------------------------------------

  timer = timerBegin(1000000);

  timerAttachInterrupt(
    timer,
    &timerISR
  );

  timerAlarm(
    timer,
    1000000,
    true,
    0
  );

  Serial.println("Hardware timer initialized.");

  // ----------------------------------------------------------
  // INITIAL MODE
  // ----------------------------------------------------------

  currentMode = AUTONOMOUS;

  Serial.println("System mode: AUTONOMOUS");

  delay(2000);

  printHelp();

  Serial.println("System ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  static unsigned long lastSensorRead = 0;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastSerialUpdate = 0;

  unsigned long currentMillis = millis();

  // ----------------------------------------------------------
  // HANDLE BUTTON
  // ----------------------------------------------------------

  handleButton();

  // ----------------------------------------------------------
  // HANDLE SERIAL COMMANDS
  // ----------------------------------------------------------

  handleSerialCommand();

  // ----------------------------------------------------------
  // READ SENSORS
  // ----------------------------------------------------------

  if (currentMillis - lastSensorRead >= SENSOR_INTERVAL)
  {
    lastSensorRead = currentMillis;

    readSensors();
  }

  // ----------------------------------------------------------
  // RUN SYSTEM MODE
  // ----------------------------------------------------------

  switch (currentMode)
  {
    case AUTONOMOUS:
      runAutonomousMode();
      break;

    case MANUAL:
      runManualMode();
      break;

    case SAFETY:
      runSafetyMode();
      break;
  }

  // ----------------------------------------------------------
  // UPDATE OLED
  // ----------------------------------------------------------

  if (currentMillis - lastDisplayUpdate >= DISPLAY_INTERVAL)
  {
    lastDisplayUpdate = currentMillis;

    updateDisplay();
  }

  // ----------------------------------------------------------
  // SERIAL STATUS
  // ----------------------------------------------------------

  if (currentMillis - lastSerialUpdate >= SERIAL_INTERVAL)
  {
    lastSerialUpdate = currentMillis;

    printSystemStatus();
  }

  // ----------------------------------------------------------
  // SMALL DELAY
  // ----------------------------------------------------------

  delay(10);
}