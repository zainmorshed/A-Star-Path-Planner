//MOTOR PINS 
#define PIN_DIRECTION_RIGHT 3
#define PIN_DIRECTION_LEFT  4
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_MOTOR_PWM_LEFT  6

//ULTRASONIC PINS 
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// MOVEMENT PARAMETERS
const int MOTOR_SPEED = 140;      
const int TURN_SPEED  = 150;

//need to TUNE
const int MOVE_TIME = 490;       
const int TURN_TIME = 495;      //90 degrees
const int NUDGE_TIME = 300;      // small nudge forward after the robto turns
const int NUDGE_SPEED = 120;     // low speed for the nudge

void setup() {
  Serial.begin(9600);

  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();
  Serial.println("READY");
  Serial.flush();
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "FORWARD") {
      moveForward();
      Serial.println("DONE");
      Serial.flush();

    } else if (cmd == "LEFT") {
      turnLeft();
      Serial.println("DONE");
      Serial.flush();

    } else if (cmd == "RIGHT") {
      turnRight();
      Serial.println("DONE");
      Serial.flush();

    } else if (cmd == "NUDGE") {
      nudgeForward();
      Serial.println("DONE");
      Serial.flush();

    } else if (cmd == "SCAN") {
      float d = stabilizedUltrasonic();
      Serial.print("DIST:");
      Serial.println(d);
      Serial.flush();

    } else if (cmd == "STOP") {
      stopMotors();
      Serial.println("DONE");
      Serial.flush();
    }
  }
}

// MOVEMENT FUNCTIONS

void moveForward() {
  // Freenove polarity: LEFT=LOW forward, RIGHT=HIGH forward
  digitalWrite(PIN_DIRECTION_LEFT, LOW);
  digitalWrite(PIN_DIRECTION_RIGHT, HIGH);

  analogWrite(PIN_MOTOR_PWM_LEFT, MOTOR_SPEED);
  analogWrite(PIN_MOTOR_PWM_RIGHT, MOTOR_SPEED);

  delay(MOVE_TIME);
  stopMotors();
}

void nudgeForward() {
  // small forward nudge after turngni
  digitalWrite(PIN_DIRECTION_LEFT, LOW);
  digitalWrite(PIN_DIRECTION_RIGHT, HIGH);

  analogWrite(PIN_MOTOR_PWM_LEFT, NUDGE_SPEED);
  analogWrite(PIN_MOTOR_PWM_RIGHT, NUDGE_SPEED);

  delay(NUDGE_TIME);
  stopMotors();
}

void turnLeft() {
  // spin left
  digitalWrite(PIN_DIRECTION_LEFT, HIGH);
  digitalWrite(PIN_DIRECTION_RIGHT, HIGH);

  analogWrite(PIN_MOTOR_PWM_LEFT, TURN_SPEED);
  analogWrite(PIN_MOTOR_PWM_RIGHT, TURN_SPEED);

  delay(TURN_TIME);
  stopMotors();
}

void turnRight() {
  // spin right
  digitalWrite(PIN_DIRECTION_LEFT, LOW);
  digitalWrite(PIN_DIRECTION_RIGHT, LOW);

  analogWrite(PIN_MOTOR_PWM_LEFT, TURN_SPEED);
  analogWrite(PIN_MOTOR_PWM_RIGHT, TURN_SPEED);

  delay(TURN_TIME);
  stopMotors();
}

// motor stop
void stopMotors() {
  analogWrite(PIN_MOTOR_PWM_LEFT, 0);
  analogWrite(PIN_MOTOR_PWM_RIGHT, 0);
  delay(200);  //delay before scanning - this shoudl
}


float stabilizedUltrasonic() {
  stopMotors();      //motors fully stop before we scan
  delay(80);         // delay

  long d1 = readUltrasonicRaw();
  delay(30);
  long d2 = readUltrasonicRaw();
  delay(30);
  long d3 = readUltrasonicRaw();

  long d = min(d1, min(d2, d3)); 
  return d;
}

float readUltrasonicRaw() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = dur * 0.034 / 2;

  if (dist == 0 || dist > 400) return 400;
  return dist;
}
