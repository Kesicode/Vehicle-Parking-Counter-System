#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

/* ================= DISPLAY ================= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ================= PINS ================= */
// IR Sensors (ACTIVE LOW)
#define IR1 2
#define IR2 3
#define IR3 4

// Ultrasonic Sensors
#define ENTRY_TRIG 5
#define ENTRY_ECHO 6
#define EXIT_TRIG 7
#define EXIT_ECHO 8

// Servos
#define ENTRY_SERVO 10
#define EXIT_SERVO 9

Servo entryGate;
Servo exitGate;

/* ================= CONSTANTS ================= */
const int TOTAL_SPOTS = 3;
const int DETECTION_DISTANCE = 10;     // cm
const int GATE_CLOSED = 0;
const int GATE_OPEN = 90;

const unsigned long CHECK_INTERVAL = 250;
const unsigned long GATE_OPEN_DURATION = 4000;
const unsigned long DEBOUNCE_DELAY = 2500;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;

/* ================= VARIABLES ================= */
int occupiedSpots = 0;
int availableSpots = TOTAL_SPOTS;
int totalEntries = 0;
int totalExits = 0;

unsigned long lastEntryCheck = 0;
unsigned long lastExitCheck = 0;
unsigned long entryGateOpenTime = 0;
unsigned long exitGateOpenTime = 0;
unsigned long lastEntryTrigger = 0;
unsigned long lastExitTrigger = 0;
unsigned long lastDisplayUpdate = 0;

enum GateState { CLOSED, OPEN };
GateState entryGateState = CLOSED;
GateState exitGateState = CLOSED;

bool entryVehicleDetected = false;
bool exitVehicleDetected = false;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(9600);
  printStartupBanner();

  pinMode(IR1, INPUT_PULLUP);
  pinMode(IR2, INPUT_PULLUP);
  pinMode(IR3, INPUT_PULLUP);

  pinMode(ENTRY_TRIG, OUTPUT);
  pinMode(ENTRY_ECHO, INPUT);
  pinMode(EXIT_TRIG, OUTPUT);
  pinMode(EXIT_ECHO, INPUT);

  entryGate.attach(ENTRY_SERVO);
  exitGate.attach(EXIT_SERVO);

  entryGate.write(GATE_CLOSED);
  exitGate.write(GATE_CLOSED);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED FAILED"));
    while (1);
  }

  display.setTextColor(SSD1306_WHITE);
  displayWelcome();
  delay(2500);
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();

  checkParkingSpots();

  if (now - lastEntryCheck >= CHECK_INTERVAL) {
    checkEntryGate();
    lastEntryCheck = now;
  }

  if (now - lastExitCheck >= CHECK_INTERVAL) {
    checkExitGate();
    lastExitCheck = now;
  }

  manageEntryGate();
  manageExitGate();

  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = now;
  }
}

/* ================= PARKING ================= */
void checkParkingSpots() {
  occupiedSpots = 0;
  if (!digitalRead(IR1)) occupiedSpots++;
  if (!digitalRead(IR2)) occupiedSpots++;
  if (!digitalRead(IR3)) occupiedSpots++;
  availableSpots = TOTAL_SPOTS - occupiedSpots;
}

/* ================= ULTRASONIC ================= */
long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

/* ================= ENTRY GATE ================= */
void checkEntryGate() {
  long d = getDistance(ENTRY_TRIG, ENTRY_ECHO);
  unsigned long now = millis();

  if (d < DETECTION_DISTANCE && availableSpots > 0) {
    if (!entryVehicleDetected && now - lastEntryTrigger > DEBOUNCE_DELAY) {
      entryVehicleDetected = true;
      lastEntryTrigger = now;
      totalEntries++;
      openEntryGate();
    }
  } else {
    entryVehicleDetected = false;
  }
}

void openEntryGate() {
  if (entryGateState == CLOSED) {
    entryGate.write(GATE_OPEN);
    entryGateState = OPEN;
    entryGateOpenTime = millis();
  }
}

void manageEntryGate() {
  if (entryGateState == OPEN &&
      millis() - entryGateOpenTime > GATE_OPEN_DURATION) {
    entryGate.write(GATE_CLOSED);
    entryGateState = CLOSED;
  }
}

/* ================= EXIT GATE ================= */
void checkExitGate() {
  long d = getDistance(EXIT_TRIG, EXIT_ECHO);
  unsigned long now = millis();

  if (d < DETECTION_DISTANCE) {
    if (!exitVehicleDetected && now - lastExitTrigger > DEBOUNCE_DELAY) {
      exitVehicleDetected = true;
      lastExitTrigger = now;
      totalExits++;
      openExitGate();
    }
  } else {
    exitVehicleDetected = false;
  }
}

void openExitGate() {
  if (exitGateState == CLOSED) {
    exitGate.write(GATE_OPEN);
    exitGateState = OPEN;
    exitGateOpenTime = millis();
  }
}

void manageExitGate() {
  if (exitGateState == OPEN &&
      millis() - exitGateOpenTime > GATE_OPEN_DURATION) {
    exitGate.write(GATE_CLOSED);
    exitGateState = CLOSED;
  }
}

/* ================= DISPLAY ================= */
void displayWelcome() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 20);
  display.println(F("SMART"));
  display.setCursor(10, 40);
  display.println(F("PARKING"));
  display.display();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Available: "));
  display.print(availableSpots);
  display.print(F("/"));
  display.print(TOTAL_SPOTS);

  display.setCursor(0, 15);
  display.print(F("Entry: "));
  display.print(entryGateState == OPEN ? "OPEN" : "CLOSED");

  display.setCursor(0, 25);
  display.print(F("Exit : "));
  display.print(exitGateState == OPEN ? "OPEN" : "CLOSED");

  display.setCursor(0, 40);
  display.print(F("IN : "));
  display.print(totalEntries);

  display.setCursor(64, 40);
  display.print(F("OUT: "));
  display.print(totalExits);

  display.display();
}

/* ================= SERIAL ================= */
void printStartupBanner() {
  Serial.println(F("SMART PARKING SYSTEM"));
  Serial.println(F("System Initializing..."));
}
