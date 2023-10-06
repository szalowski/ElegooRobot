#include <Servo.h>
Servo myservo;

// Servo-Motor für "Augen"
int minAngle = 700;   //the pulse width, in microseconds, corresponding to the minimum (0-degree) angle on the servo (defaults to 700)
int maxAngle = 2400;  //the pulse width, in microseconds, corresponding to the maximum (180-degree) angle on the servo (defaults to 2400)

// Ultraschall-Sensoeren --> "Augen"
int Echo = A4;  // Hörer-Auge
int Trig = A5;  // Sprecher-Auge
unsigned long int distance = 0;   // Gemessener Abstand zum nächsten Hinderniss

// Thread Support
static unsigned long int threadServoMillis = 0;
static unsigned long int threadMotorMillis = 0;

// PIN-Belegung definieren ... Welcher PIN steuert welches Bauteil.
#define ENA 5     // Strom linke Motoren      (An / Aus / Geschwindigkeit   ... digital HIGH / digital LOW / analog 0 bis 255)
#define IN1 7     // Steuerung linke Motoren  (Vorwärts / Rückwärts / Halt  ... 0 / 1 / 0 oder 1)
#define IN2 8     // Steuerung linke Motoren  (Vorwärts / Rückwärts / Halt  ... 1 / 0 / 0 oder 1)

#define ENB 6     // Strom rechte Motoren     (An / Aus / Geschwindigkeit   ... digital HIGH / digital LOW / analog 0 bis 255)
#define IN3 9     // Steuerung rechte Motoren (Vorwärts / Rückwärts / Halt  ... 1 / 0 / 0 oder 1)
#define IN4 11    // Steuerung rechte Motoren (Vorwärts / Rückwärts / Halt  ... 0 / 1 / 0 oder 1)

#define MAX_DIST_FRONT 40   // max. Abstand für Ultraschall-Steuerung
#define MAX_DIST_SIDE 20   // max. Abstand für Ultraschall-Steuerung
#define THREAD_SERVO_MILLIS 300 // Millisekunden bis zur nächsten Messung
#define THREAD_MOTOR_MILLIS 2500 // Millisekunden bis zur nächsten Halt

// Hilfsfunktionen ... einmal programmieren und dann immer wieder verwenden, wenn benötigt.
void turnOffMotorLeft() {
  analogWrite(ENA, 0);
  digitalWrite(ENA, LOW);
}
void turnOffMotorRight() {
  analogWrite(ENB, 0);
  digitalWrite(ENB, LOW);
}
void turnOnMotorLeftDig() {
  digitalWrite(ENA, HIGH);
}
void turnOnMotorRightDig() {
  digitalWrite(ENB, HIGH);
}
void turnOnMotorLeftAna(int v) {
  analogWrite(ENA, v % 256);
}
void turnOnMotorRightAna(int v) {
  analogWrite(ENB, v % 256);
}
// turnOffMotors ... Beide Motoren: Strom Aus
void turnOffMotors() {
  turnOffMotorLeft();
  turnOffMotorRight();
}
// turnOnMotors ... Beide Motoren: Strom An
void turnOnMotors() {
  turnOnMotorLeftDig();
  turnOnMotorRightDig();
}
// turnOnMotors ... Beide Motoren: Strom An mit Geschwindigkeit v
void turnOnMotorsAna(int v) {
  turnOnMotorLeftAna(v);
  turnOnMotorRightAna(v);
}
// leftGoForward() ... linke Motoren sollen sich vorwärts bewegen
void leftGoForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}
// leftGoBackward() ... linke Motoren sollen sich rückwärts bewegen
void leftGoBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
// rightGoForward() ... rechte Motoren sollen sich vorwärts bewegen
void rightGoForward() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
// rightGoBackward() ... rechte Motoren sollen sich rückwärts bewegen
void rightGoBackward() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void turnOnMotorsForTurn() {
  // Nicht ändern!!! ... Passt in etwa so für den Laminatboden zu Hause
  turnOnMotorsAna(160);
  delay(512);
}
void turnRight() {
  turnOffMotors();
  rightGoBackward();
  leftGoForward();
  turnOnMotors();
  delay(random(200, 500));
}
void turnLeft() {
  turnOffMotors();
  leftGoBackward();
  rightGoForward();
  turnOnMotors();
  delay(random(200, 500));
}
// turnRight90 ... Das Auto dreht sich auf der Stelle um 90 Grad nach rechts.
void turnRight90() {
  turnOffMotors();
  rightGoBackward();
  leftGoForward();
  turnOnMotorsForTurn();
}
// turnLeft90 ... Das Auto dreht sich auf der Stelle um 90 Grad nach links.
void turnLeft90() {
  turnOffMotors();
  leftGoBackward();
  rightGoForward();
  turnOnMotorsForTurn();
}
// goStepForward() ... Das Auto bewegt sich einen gegebenen Zeitraum ms vorwärts
void goStepForward(int ms) {
  turnOffMotors();
  rightGoForward();
  leftGoForward();
  turnOnMotors();
  delay(ms);
  turnOffMotors();
}
// goStepBackward() ... Das Auto bewegt sich einen gegebenen Zeitraum ms rückwärts
void goStepBackward(int ms) {
  turnOffMotors();
  rightGoBackward();
  leftGoBackward();
  turnOnMotors();
  delay(ms);
  turnOffMotors();
}
void goForward() {
  rightGoForward();
  leftGoForward();
  turnOnMotorsAna(100);
}
void goBackward() {
  rightGoBackward();
  leftGoBackward();
  turnOnMotorsAna(100);
}
//Ultrasonic distance measurement Sub function ... Kopiert vom Sketch "Obstacle_Avoidance_Car"
int Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}  

//init the car
void setup() {
  // initialize ultra sonic servo motor
  myservo.attach(3,minAngle,maxAngle);  //setting the servo IO pin and the steering range.
  myservo.write(100);                   // move servo to center position -> 90° ... bei uns schief verbaut --> deshalb 100°

  // initialize ultra sonic sensors
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  

  // initialize motors
  // set IO pin mode OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void resetUltraSonic () {
  myservo.write(100);                   // move servo to center position -> 90° ... bei uns schief verbaut --> deshalb 100°
}
void lookForward() {
  resetUltraSonic();
}
void lookLeft() {
  myservo.write(160);
}
void lookRight() {
  myservo.write(40);
}
int measure() {
  return Distance_test();
}

void thread_runMotors() {
static enum {ON, OFF} motor = OFF;
  switch(motor) {
    case ON:
      if ((millis() - threadMotorMillis) > THREAD_MOTOR_MILLIS) {
        turnOffMotors();
        motor = OFF;
        threadMotorMillis = millis();
      }
      break;
    case OFF:
      if ((millis() - threadMotorMillis) > THREAD_MOTOR_MILLIS) {
        goForward();
        motor = ON;
        threadMotorMillis = millis();
      }
      break;
    default:
      motor = ON;
      break;
  }
}

void thread_runUltraSonic() {

static enum {LEFT, FORWARD1, FORWARD2, RIGHT} servo = FORWARD1;   // Servo Zustände, LEFT ... Links, FORWARD1 ... Geradeaus von Rechts kommend, FORWARD2 ... Geradeaus von Links kommend, RIGHT ... Rechts

  switch(servo) {
    case LEFT:
      lookLeft();
      if ((millis() - threadServoMillis) > THREAD_SERVO_MILLIS) {
        distance = measure();
        if (distance <= MAX_DIST_SIDE) {
          turnRight();
        }
        servo = FORWARD2;
        threadServoMillis = millis();

      }
      break;
    case FORWARD1:
      lookForward();
      if ((millis() - threadServoMillis) > THREAD_SERVO_MILLIS) {
        distance = measure();
        if (distance <= MAX_DIST_FRONT) {
          turnLeft();
        }
        servo = LEFT;
        threadServoMillis = millis();
      }
      break;
    case FORWARD2:
      lookForward();
      if ((millis() - threadServoMillis) > THREAD_SERVO_MILLIS) {
        distance = measure();
        if (distance <= MAX_DIST_FRONT) {
          turnRight();
        }
        servo = RIGHT;
        threadServoMillis = millis();
      }
      break;
    case RIGHT:
      lookRight();
      if ((millis() - threadServoMillis) > THREAD_SERVO_MILLIS) {
        distance = measure();
        if (distance <= MAX_DIST_SIDE) {
          turnLeft();
        }
        servo = FORWARD1;
        threadServoMillis = millis();
      }
      break;
    default:
      servo = FORWARD1;
      break;
  }
}

//main loop
void loop() {

//  thread_runMotors();
//  thread_runUltraSonic();

  distance = Distance_test();
  Serial.println(distance);
}
