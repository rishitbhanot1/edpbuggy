// =====================================
// BUGGY 1 - GOLD CHALLENGE
// Clockwise
// Starts first
// Waits at center/lower gantry
// Resumes after coordinator sends R1
// Counts loops at RIGHT gantry after resume
// Parks by taking RIGHT inward parking lane
// =====================================

// ---------- MOTOR PINS ----------
const int L1 = 5;
const int L2 = 6;
const int R1 = 7;
const int R2 = 8;

// ---------- LINE SENSORS ----------
const int LS = A0;
const int RS = A1;

// ---------- GANTRY RECEIVER ----------
const int gantryPin = 4;

// ---------- ULTRASONIC ----------
const int trigPin = 13;
const int echoPin = 12;

// ---------- CONSTANTS ----------
const int OBSTACLE_STOP_DISTANCE_CM = 15;
const unsigned long GANTRY_STOP_DELAY = 1000;

// Tune these on hardware
const unsigned long EXIT_FORWARD_TIME = 220;
const unsigned long EXIT_RIGHT_TIME   = 260;
const unsigned long PARK_SETTLE_TIME  = 900;

// Gantry pulse ranges - tune if needed
const unsigned long G1_MIN = 500;
const unsigned long G1_MAX = 1500;   // left
const unsigned long G2_MIN = 1500;
const unsigned long G2_MAX = 2500;   // right
const unsigned long G3_MIN = 2500;
const unsigned long G3_MAX = 3500;   // center/lower

// ---------- STATES ----------
enum BuggyState {
  WAIT_START,
  EXIT_PARKING,
  FOLLOW_TO_CENTER,
  WAIT_AT_CENTER,
  FOLLOW_AFTER_RESUME,
  PARKING_RIGHT,
  PARKED
};

BuggyState state = WAIT_START;

// ---------- GLOBALS ----------
bool obstacleFlag = false;
bool gantryLatched = false;
bool parked = false;
bool resumedOnce = false;
bool parkingArmed = false;

unsigned long stateStartTime = 0;
long duration = 0;
int distanceCm = 0;

int gantryCount = 0;
int currentGantryId = 0;

int targetLoops = 1;
int completedLoops = 0;

// ---------- FUNCTION PROTOTYPES ----------
void stopMotors();
void forward();
void turnLeft();
void turnRight();

void followCW(int left, int right);
void biasRightFollow(int left, int right);

int getDistance();
void checkObstacle();
int detectGantryId();
void checkGantry();

void handleSerial();
void processCommand(String cmd);
void sendMsg(String msg);

// ---------- SETUP ----------
void setup() {
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  pinMode(LS, INPUT_PULLUP);
  pinMode(RS, INPUT_PULLUP);

  pinMode(gantryPin, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
  stopMotors();
}

// ---------- LOOP ----------
void loop() {
  handleSerial();

  if (parked) {
    stopMotors();
    return;
  }

  if (state == WAIT_START) {
    stopMotors();
    return;
  }

  checkObstacle();
  if (obstacleFlag) return;

  checkGantry();

  int left = digitalRead(LS);
  int right = digitalRead(RS);

  switch (state) {
    case EXIT_PARKING:
      if (millis() - stateStartTime < EXIT_FORWARD_TIME) {
        forward();
      } else if (millis() - stateStartTime < EXIT_FORWARD_TIME + EXIT_RIGHT_TIME) {
        turnRight();
      } else {
        state = FOLLOW_TO_CENTER;
      }
      break;

    case FOLLOW_TO_CENTER:
      followCW(left, right);
      break;

    case WAIT_AT_CENTER:
      stopMotors();
      break;

    case FOLLOW_AFTER_RESUME:
      if (parkingArmed) {
        state = PARKING_RIGHT;
        stateStartTime = millis();
        sendMsg("<B1_PARK_ENTRY_RIGHT>");
      } else {
        followCW(left, right);
      }
      break;

    case PARKING_RIGHT:
      if (millis() - stateStartTime < PARK_SETTLE_TIME) {
        biasRightFollow(left, right);
      } else {
        stopMotors();
        parked = true;
        state = PARKED;
        sendMsg("<B1_PARKED>");
      }
      break;

    case PARKED:
      stopMotors();
      break;

    default:
      stopMotors();
      break;
  }
}

// ---------- SERIAL / XBEE ----------
void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() > 0) {
    processCommand(cmd);
  }
}

void processCommand(String cmd) {
  if (cmd.startsWith("S1,")) {
    if (state == WAIT_START) {
      int commaIndex = cmd.indexOf(',');
      String loopText = cmd.substring(commaIndex + 1);
      int parsedLoops = loopText.toInt();
      if (parsedLoops > 0) {
        targetLoops = parsedLoops;
      } else {
        targetLoops = 1;
      }

      state = EXIT_PARKING;
      stateStartTime = millis();
      sendMsg("<B1_START," + String(targetLoops) + ">");
    }
  } else if (cmd == "R1") {
    if (state == WAIT_AT_CENTER) {
      resumedOnce = true;
      state = FOLLOW_AFTER_RESUME;
      sendMsg("<B1_RESUME>");
    }
  }
}

void sendMsg(String msg) {
  Serial.println(msg);
}

// ---------- OBSTACLE ----------
void checkObstacle() {
  distanceCm = getDistance();

  if (distanceCm > 0 && distanceCm <= OBSTACLE_STOP_DISTANCE_CM) {
    if (!obstacleFlag) {
      obstacleFlag = true;
      stopMotors();
      sendMsg("<B1_OBS>");
    }
  } else {
    if (obstacleFlag) {
      obstacleFlag = false;
      sendMsg("<B1_CLEAR>");
    }
  }
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 0;

  return duration * 0.034 / 2;
}

// ---------- GANTRY ----------
void checkGantry() {
  int gState = digitalRead(gantryPin);

  if (gState == HIGH && !gantryLatched) {
    int gId = detectGantryId();
    if (gId != 0) {
      gantryLatched = true;
      currentGantryId = gId;
      gantryCount++;

      stopMotors();
      delay(GANTRY_STOP_DELAY);

      if (gId == 1) sendMsg("<B1_G1_LEFT>");
      else if (gId == 2) sendMsg("<B1_G2_RIGHT>");
      else if (gId == 3) sendMsg("<B1_G3_CENTER>");

      // Initial center wait
      if (!resumedOnce && gId == 3 && state == FOLLOW_TO_CENTER) {
        state = WAIT_AT_CENTER;
        sendMsg("<B1_G3_CENTER_WAIT>");
        return;
      }

      // After resume, count loops at RIGHT gantry
      if (resumedOnce && gId == 2 && state == FOLLOW_AFTER_RESUME) {
        completedLoops++;
        sendMsg("<B1_LOOP," + String(completedLoops) + ">");
        if (completedLoops >= targetLoops) {
          parkingArmed = true;
          sendMsg("<B1_PARK_ARM>");
        }
      }
    }
  }

  if (gState == LOW) {
    gantryLatched = false;
  }
}

int detectGantryId() {
  unsigned long d = pulseIn(gantryPin, HIGH, 5000);

  if (d > G1_MIN && d < G1_MAX) return 1;
  if (d >= G2_MIN && d < G2_MAX) return 2;
  if (d >= G3_MIN && d < G3_MAX) return 3;

  return 0;
}

// ---------- LINE FOLLOW ----------
void followCW(int left, int right) {
  if (left == 1 && right == 1) {
    forward();
  } else if (left == 1 && right == 0) {
    turnLeft();
  } else if (left == 0 && right == 1) {
    turnRight();
  } else {
    forward();
  }
}

void biasRightFollow(int left, int right) {
  if (left == 1 && right == 1) {
    turnRight();
  } else if (left == 1 && right == 0) {
    turnLeft();
  } else if (left == 0 && right == 1) {
    turnRight();
  } else {
    forward();
  }
}

// ---------- MOTOR CONTROL ----------
void forward() {
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void turnRight() {
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
}

void turnLeft() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void stopMotors() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
}
