// =====================================
// BUGGY 2 - GOLD CHALLENGE
// Command format:
//   S2n  -> start Buggy 2 with n loops, example: S23
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

// Tune on hardware
const unsigned long EXIT_FORWARD_TIME = 220;
const unsigned long EXIT_LEFT_TIME    = 260;
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
  FOLLOW_ROUTE,
  PARKING_LEFT,
  PARKED
};

BuggyState state = WAIT_START;

// ---------- GLOBALS ----------
bool obstacleFlag = false;
bool gantryLatched = false;
bool parked = false;
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

void followACW(int left, int right);
void biasLeftFollow(int left, int right);

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
      } else if (millis() - stateStartTime < EXIT_FORWARD_TIME + EXIT_LEFT_TIME) {
        turnLeft();
      } else {
        state = FOLLOW_ROUTE;
      }
      break;

    case FOLLOW_ROUTE:
      if (parkingArmed) {
        state = PARKING_LEFT;
        stateStartTime = millis();
        sendMsg("<B2_PARK_ENTRY_LEFT>");
      } else {
        followACW(left, right);
      }
      break;

    case PARKING_LEFT:
      if (millis() - stateStartTime < PARK_SETTLE_TIME) {
        biasLeftFollow(left, right);
      } else {
        stopMotors();
        parked = true;
        state = PARKED;
        sendMsg("<B2_PARKED>");
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
  // Start format: S2n, e.g. S23 means Buggy 2 start with 3 loops
  if (cmd.length() >= 3 && cmd.charAt(0) == 'S' && cmd.charAt(1) == '2') {
    if (state == WAIT_START) {
      char loopChar = cmd.charAt(2);
      if (loopChar >= '1' && loopChar <= '9') {
        targetLoops = loopChar - '0';
      } else {
        targetLoops = 1;
      }

      completedLoops = 0;
      parkingArmed = false;
      parked = false;
      gantryCount = 0;

      state = EXIT_PARKING;
      stateStartTime = millis();
      sendMsg("<B2_START_" + String(targetLoops) + ">");
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
      sendMsg("<B2_OBS>");
    }
  } else {
    if (obstacleFlag) {
      obstacleFlag = false;
      sendMsg("<B2_CLEAR>");
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

      if (gId == 1) sendMsg("<B2_G1_LEFT>");
      else if (gId == 2) sendMsg("<B2_G2_RIGHT>");
      else if (gId == 3) sendMsg("<B2_G3_CENTER>");

      // Count loops at LEFT gantry
      if (gId == 1 && state == FOLLOW_ROUTE) {
        completedLoops++;
        sendMsg("<B2_LOOP_" + String(completedLoops) + ">");

        if (completedLoops >= targetLoops) {
          parkingArmed = true;
          sendMsg("<B2_PARK_ARM>");
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
void followACW(int left, int right) {
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

void biasLeftFollow(int left, int right) {
  if (left == 1 && right == 1) {
    turnLeft();
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
