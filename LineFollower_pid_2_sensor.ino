#include <Arduino.h>

const int leftLineSensor  = 2;
const int rightLineSensor = 3;
const int obstacleSensor  = 4;

const int motorA1 = 5;
const int motorA2 = 6;
const int motorB1 = 9;
const int motorB2 = 10;

const int statusLED = 13;

float Kp = 110;
float Kd = 0;
float Ki = 0;

int baseSpeed = 120;
int maxSpeed  = 200;

int lastError = 0;
float integral = 0;

#define ROBOT_ID 1

const unsigned long ELECTION_TIME_MS = 4000;
const unsigned long BROADCAST_PERIOD_MS = 200;

const int LEADER_STOP_DISTANCE_CM = 12;
const int LEADER_LOST_TIMEOUT_MS = 1500;

struct PeerInfo {
  bool seen = false;
  int minAngleDeg = 9999;
};

struct Detection {
  bool valid = false;
  int id = -1;
  int bearingDeg = 0;
  int distCm = 999;
};

PeerInfo peer[4];
Detection det[4];

enum SwarmState {
  STATE_ELECT = 0,
  STATE_LEADER,
  STATE_FOLLOWER
};

SwarmState state = STATE_ELECT;

unsigned long tStart = 0;
unsigned long tLastBroadcast = 0;
unsigned long tLeaderLastSeen = 0;

int electedLeaderID = -1;

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void drive(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  analogWrite(motorA1, leftSpeed);
  digitalWrite(motorA2, LOW);

  analogWrite(motorB1, rightSpeed);
  digitalWrite(motorB2, LOW);
}

void lineFollowStep() {
  int leftVal  = digitalRead(leftLineSensor);
  int rightVal = digitalRead(rightLineSensor);

  int error = 0;

  if (leftVal == LOW && rightVal == LOW) error = 0;
  else if (leftVal == HIGH && rightVal == LOW) error = -1;
  else if (leftVal == LOW && rightVal == HIGH) error = 1;
  else error = 0;

  int derivative = error - lastError;
  integral += error;

  int correction = (Kp * error) + (Kd * derivative) + (Ki * integral);
  lastError = error;

  int leftMotorSpeed  = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  drive(leftMotorSpeed, rightMotorSpeed);
}

void broadcastMyMinAngle(int myMinAngleDeg) {
  Serial.print("M,");
  Serial.print(ROBOT_ID);
  Serial.print(",");
  Serial.println(myMinAngleDeg);
}

void readPeerMessages() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() < 5) continue;

    if (line.charAt(0) == 'M') {
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      if (c1 < 0 || c2 < 0) continue;

      int id = line.substring(c1 + 1, c2).toInt();
      int val = line.substring(c2 + 1).toInt();

      if (id >= 1 && id <= 3 && id != ROBOT_ID) {
        peer[id].seen = true;
        peer[id].minAngleDeg = val;
      }
    }
  }
}

void clearDetections() {
  for (int i = 1; i <= 3; i++) {
    det[i].valid = false;
    det[i].id = i;
    det[i].bearingDeg = 0;
    det[i].distCm = 999;
  }
}

bool readCameraDetectionsFromSerial(Stream &cam) {
  if (!cam.available()) return false;

  String s = cam.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return false;

  clearDetections();

  int start = 0;
  while (start < s.length()) {
    int sep = s.indexOf(';', start);
    String token = (sep == -1) ? s.substring(start) : s.substring(start, sep);

    token.trim();
    if (token.length() >= 6 && token.charAt(0) == 'R') {
      int colon = token.indexOf(':');
      int comma = token.indexOf(',');
      if (colon > 1 && comma > colon) {
        int id = token.substring(1, colon).toInt();
        int bearing = token.substring(colon + 1, comma).toInt();
        int dist = token.substring(comma + 1).toInt();

        if (id >= 1 && id <= 3 && id != ROBOT_ID) {
          det[id].valid = true;
          det[id].id = id;
          det[id].bearingDeg = bearing;
          det[id].distCm = dist;
        }
      }
    }

    if (sep == -1) break;
    start = sep + 1;
  }

  return true;
}

int computeMyMinAngle() {
  int best = 9999;
  for (int i = 1; i <= 3; i++) {
    if (i == ROBOT_ID) continue;
    if (!det[i].valid) continue;
    int a = abs(det[i].bearingDeg);
    if (a < best) best = a;
  }
  return best;
}

int chooseLeaderFromAll(int myMinAngle) {
  int leader = ROBOT_ID;
  int leaderVal = myMinAngle;

  for (int i = 1; i <= 3; i++) {
    if (i == ROBOT_ID) continue;
    if (!peer[i].seen) continue;

    int v = peer[i].minAngleDeg;
    if (v < leaderVal) {
      leaderVal = v;
      leader = i;
    } else if (v == leaderVal && i < leader) {
      leader = i;
    }
  }

  return leader;
}

bool leaderIsClose() {
  if (electedLeaderID < 1 || electedLeaderID > 3) return false;
  if (!det[electedLeaderID].valid) return false;
  tLeaderLastSeen = millis();
  return (det[electedLeaderID].distCm <= LEADER_STOP_DISTANCE_CM);
}

void setup() {
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
  pinMode(obstacleSensor, INPUT);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);

  Serial.begin(9600);

  tStart = millis();
}

void loop() {
  if (digitalRead(obstacleSensor) == LOW) {
    stopMotors();
    return;
  }

  readCameraDetectionsFromSerial(Serial);
  readPeerMessages();

  unsigned long now = millis();

  if (state == STATE_ELECT) {
    baseSpeed = 90;
    lineFollowStep();

    int myMinAngle = computeMyMinAngle();

    if (now - tLastBroadcast >= BROADCAST_PERIOD_MS) {
      tLastBroadcast = now;
      broadcastMyMinAngle(myMinAngle);
    }

    if (now - tStart >= ELECTION_TIME_MS) {
      electedLeaderID = chooseLeaderFromAll(myMinAngle);

      if (electedLeaderID == ROBOT_ID) {
        state = STATE_LEADER;
        digitalWrite(statusLED, HIGH);
        stopMotors();
      } else {
        state = STATE_FOLLOWER;
        digitalWrite(statusLED, LOW);
        tLeaderLastSeen = now;
      }
    }
  } 
  else if (state == STATE_LEADER) {
    stopMotors();
    digitalWrite(statusLED, HIGH);
  } 
  else if (state == STATE_FOLLOWER) {
    baseSpeed = 120;

    if (leaderIsClose()) {
      stopMotors();
      return;
    }

    if (now - tLeaderLastSeen > (unsigned long)LEADER_LOST_TIMEOUT_MS) {
    }

    lineFollowStep();
  }
}
