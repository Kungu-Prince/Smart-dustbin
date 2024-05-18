#include <LiquidCrystal.h>
#include <Servo.h>

// Define the pins
const int pirSensorPin = 2;
const int ultrasonicTrigPin = 3;
const int ultrasonicEchoPin = 4;
const int servoPin = 9;

// Initialize the LCD with the pins
LiquidCrystal lcd(A5, A4, A3, A2, A1, A0);

// Create a servo object
Servo lidServo;

// Define constants and variables
const int maxGarbageCapacity = 100;
int currentGarbageLevel = 0;
bool isLidOpen = false;
unsigned long lastMotionDetected = 0; // Time when the last motion was detected
bool isFullAlertSent = false; // Flag to track if the full alert is sent

// Virtual GPS coordinates/location
float binLatitude = 0.0;
float binLongitude = 0.0;

void setup() {
  // Initialize pins
  pinMode(pirSensorPin, INPUT);
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);
  
  // Initialize the servo
  lidServo.attach(servoPin);
  closeLid(); // Ensure lid is closed initially
  
  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.print("Smart Dustbin");
  delay(2000); // Display initial message for 2 seconds
}

void loop() {
  int pirSensorValue = digitalRead(pirSensorPin);
  int ultrasonicDistance = getUltrasonicDistance();
  
  if (pirSensorValue == HIGH && !isLidOpen) {
    openLid();
    lastMotionDetected = millis(); // Update the time of the last detected motion
    delay(5000);
  }
  if (pirSensorValue == LOW && isLidOpen && millis() - lastMotionDetected > 5000) { // If no motion is detected for 5 seconds
    closeLid();
    delay(5000);
  }

  if (ultrasonicDistance < 200) {
    currentGarbageLevel = map(ultrasonicDistance, 200, 0, 0, maxGarbageCapacity);
  }

  int percentage = map(currentGarbageLevel, 0, maxGarbageCapacity, 0, 100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Garbage Level:");
  lcd.setCursor(0, 1);
  lcd.print(percentage);
  lcd.print("%");

  if (percentage >= 80 && !isFullAlertSent) {
    sendFullAlert(); // Trigger full alert when garbage level is above 80%
    isFullAlertSent = true; // Set flag to indicate alert sent
  }
  
  delay(1000); // Delay for stability
}

int getUltrasonicDistance() {
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);
  
  long duration = pulseIn(ultrasonicEchoPin, HIGH);
  int distance = duration * 0.034 / 2; // Convert to centimeters
  return distance;
}

void openLid() {
  lidServo.write(90); // Adjust angle as needed
  isLidOpen = true;
}

void closeLid() {
  lidServo.write(0); // Adjust angle as needed
  isLidOpen = false;
}

// Function to simulate sending full alert with GPS coordinates/location
void sendFullAlert() {
  // Simulate GPS coordinates/location
  binLatitude = 51.5074; // London latitude
  binLongitude = -0.1278; // London longitude
  
  // Display alert message with GPS coordinates/location
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bin is full!");
  lcd.setCursor(0, 1);
  lcd.print("Latitude: ");
  lcd.print(binLatitude, 6); // Display latitude with 6 decimal places
  lcd.setCursor(0, 2);
  lcd.print("Longitude: ");
  lcd.print(binLongitude, 6);// Display longitude with 6 decimal places
  delay(7000);
}