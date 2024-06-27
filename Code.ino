#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Servo.h>

int mq3Pin = A0;
int buzzerPin = 9;
int ledPin = 10;
int alcoholThreshold = 200; // Alcohol threshold is 200ppm
int pot = A3; //Speed sensor
int sensorValue;
int Speed;
int ultrasonicTriggerPin = A1; // Pin connected to the ultrasonic sensor's trigger pin
int ultrasonicEchoPin = A2; // Pin connected to the ultrasonic sensor's echo pin
float speedOfSound = 0.3432; // Speed of sound in km/s 
float stoppingDistanceMultiplier = 2.5; // Multiplier to calculate stopping distance
float stoppingDistance; // Stopping distance based on speed
LiquidCrystal_I2C lcd(0x27, 20, 4);

const byte ROWS = 4;
const byte COLS = 3;

unsigned long startTime = 0;
bool drowsy = false;


char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {5, 4, 3, 2};
byte colPins[COLS] = {8, 7, 6};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

Servo servoMotor;

void setup() {
  lcd.begin(20, 4);
  pinMode(buzzerPin,OUTPUT);
  pinMode(ledPin,OUTPUT);
  pinMode(pot,INPUT);
  pinMode(ultrasonicTriggerPin,OUTPUT);
  pinMode(ultrasonicEchoPin,INPUT);
  
  lcd.print("Alcohol Detector");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  delay(700);
  
  servoMotor.attach(12); // The servo attached to pin 11 
  servoMotor.write(0); // Rotate the servo to 0 degrees (initial position)
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  float alcoholLevel = analogRead(mq3Pin);
  
  if (alcoholLevel > alcoholThreshold) {
    lcd.clear();
    lcd.print("ALCOHOL LIMIT EXCEEDED!");
    lcd.setCursor(0, 1);
    lcd.print("Last drink in 3 hrs?");
    lcd.setCursor(0, 2);
    lcd.print("1-Yes  2-No");
    
    char key = keypad.getKey();
    
    if (key == '1') {
      lcd.clear();
      lcd.print("Unsafe to drive!");
      digitalWrite(buzzerPin, HIGH);
      tone(13, 200);
      digitalWrite(ledPin, HIGH);
      servoMotor.write(180); // Rotate the servo to 180 degrees
      delay(1000);
    } else if (key == '2') {
      lcd.clear();
      lcd.print("Drive safely and observe traffic laws!");
      digitalWrite(buzzerPin, LOW);
      noTone(13);
      digitalWrite(ledPin, LOW);
      servoMotor.write(0); // Servo motor mantains start position
      delay(1000);
    }
  } else {
    lcd.clear();
    lcd.print("Drive safely");
    digitalWrite(buzzerPin, LOW);
    noTone(13);
    digitalWrite(ledPin, LOW);
    servoMotor.write(0); // Ensuring that the servo is at the initial position
  }
  
  delay(3000); // Adjust this delay according to your needs
  SpeedSensor();
  UltrasonicSensor();

}

void SpeedSensor() {
  sensorValue = analogRead(pot); // Read potentiometer value
  float Speed = map(sensorValue,0,1023,0,200); // Map to speed range

  Serial.print("Vehicle Speed (km/h): ");// Print the mapped speed to the serial monitor
  Serial.println(Speed);
  delay(1000);

  delay(100); // Delay for stability
}

void UltrasonicSensor() {
  digitalWrite(ultrasonicTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, LOW);

  float duration = pulseIn(ultrasonicEchoPin, HIGH); //  // Measuring the echo duration
  float distance = (duration * speedOfSound) / (2);
  Speed = analogRead(pot);
  Speed = map(analogRead(pot), 0, 1023, 0, 220);// Map to speed range
  stoppingDistance = (Speed*Speed)/(2*4*1.72);
  
  if ((distance-1) <= stoppingDistance)   //The purpose of subtracting 1 from distance is for safety margin.
  {
    servoMotor.write(180); // Rotate the servo to 180 degrees
    digitalWrite(buzzerPin, HIGH);
    tone(13, 200);
    digitalWrite(ledPin, HIGH);
    lcd.clear();
    lcd.print("Critical situation");
    lcd.setCursor(0, 1);
    lcd.print("Vehicle stopping!!");
  } else {
    servoMotor.write(0); // Rotate the servo to 0 degrees
  }
  
  // Print the measured distance and stopping distance to the serial monitor
  Serial.print("Measured Distance: ");
  Serial.print(distance);
  Serial.println(" km");
  Serial.print("Stopping Distance: ");
  Serial.print(stoppingDistance);
  Serial.println(" km");
  
  delay(100); // Delay for stability
    Driverstate();
}

void Driverstate() {
    if (Serial.available()) {
        char receivedChar = Serial.read();
        if (receivedChar == 'D') {
            // Driver is drowsy
            drowsy = true;
            digitalWrite(buzzerPin, HIGH);
            digitalWrite(ledPin, HIGH);
            lcd.clear();
            lcd.print("Fatigue detected, Take a rest!!");
            startTime = millis(); // Start the timer
        } 
    }

    // Check if 10 seconds have passed
    if (drowsy && millis() - startTime >= 10000) {
        // Activate servo motor
        lcd.clear();
        lcd.print("Critical sistuation, Vehicle stopping");
        servoMotor.write(180); // Rotate the servo to 180 degrees
    }
}
