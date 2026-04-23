// =========================
// MOTOR PINS
// =========================
const int L1 = 5;
const int L2 = 6;
const int R1 = 7;
const int R2 = 8;

// =========================
// LINE SENSORS
// =========================
const int LS = A0;
const int RS = A1;

// =========================
// GANTRY RECEIVER INPUT
// Photodiode receiver circuit on D4
// =========================
const int gantryPin = 4;

// =========================
// ULTRASONIC SENSOR
// =========================
const int trigPin = 13;
const int echoPin = 12;

// =========================
// CHALLENGE CONSTANTS
// =========================
const int OBSTACLE_STOP_DISTANCE_CM = 15;
const unsigned long GANTRY_STOP_DELAY = 1000;   // 1 second as required

// Exit-from-parking timings
const unsigned long EXIT_FORWARD_TIME = 220;
const unsigned long EXIT_RIGHT_TIME   = 260;    // clockwise exit; tune this

// Parking timings
const unsigned long PARK_FORWARD_1 = 180;
const unsigned long PARK_RIGHT_1   = 260;       // inward parking turn for clockwise case; tune this
const unsigned long PARK_FORWARD_2 = 260;

// =========================
// GLOBAL STATE
// =========================
long duration = 0;
int distance = 0;

bool startFlag = false;
bool obstacleFlag = false;
bool parked = false;

// Exit state
bool exitParkingMode = true;
int exitStep = 0;
unsigned long exitStepTime = 0;

// Gantry / loop state
bool gantryFlag = false;
int gantryCount = 0;
int loopCount = 0;
int currentGantryId = 0;   // 1 = left, 2 = right, 3 = center/top

// Parking state
bool parkingMode = false;
int parkingStep = 0;
unsigned long parkingStepTime = 0;

// =========================
// FUNCTION PROTOTYPES
// =========================
void stopMotors();
void forward();
void turnLeft();
void turnRight();
int getDistance();

void sendStatus(const char* buggyState, const char* trackState);
void checkObstacle();
void checkGantry();
int detectGantryId();

void handleInitialExit();
void handleMainTrack(int left, int right);
void handleParking(int left, int right);

// =========================
// SETUP
// =========================
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
    sendStatus("WAITING_FOR_START", "PARKING_START");
}

// =========================
// MAIN LOOP
// =========================
void loop() {
    // One-time wireless start command
    if (!startFlag && Serial.available()) {
        char cmd = Serial.read();

        // Use G to match common Bronze/XBee examples
        if (cmd == 'G') {
            startFlag = true;
            exitParkingMode = true;
            exitStep = 0;
            exitStepTime = millis();
            sendStatus("STARTED", "EXITING_PARKING");
        }
    }

    if (!startFlag) {
        stopMotors();
        return;
    }

    if (parked) {
        stopMotors();
        return;
    }

    // obstacle handling
    checkObstacle();
    if (obstacleFlag) {
        return;
    }

    // initial exit from parking lane to main track
    if (exitParkingMode) {
        handleInitialExit();
        return;
    }

    // gantry identification and stop/reporting
    checkGantry();

    // line sensors
    int left = digitalRead(LS);
    int right = digitalRead(RS);

    // after right gantry second time, enter parking
    if (parkingMode) {
        handleParking(left, right);
        return;
    }

    // main track follow
    handleMainTrack(left, right);
}

// =========================
// OBSTACLE HANDLING
// =========================
void checkObstacle() {
    distance = getDistance();

    if (distance > 0 && distance <= OBSTACLE_STOP_DISTANCE_CM) {
        if (!obstacleFlag) {
            obstacleFlag = true;
            stopMotors();
            sendStatus("OBSTACLE_STOP", parkingMode ? "PARKING_APPROACH" : "MAIN_TRACK");
        }
    } else {
        if (obstacleFlag) {
            obstacleFlag = false;
            sendStatus("OBSTACLE_CLEARED_RESUMING", parkingMode ? "PARKING_APPROACH" : "MAIN_TRACK");
        }
    }
}

// =========================
// GANTRY DETECTION
// Uses D4 receiver circuit / photodiode pulse width
// Suggested classification:
//  500-1500  -> Gantry 1 (left)
// 1500-2500  -> Gantry 2 (right)
// 2500-3500  -> Gantry 3 (center/top)
// Adjust thresholds if your hardware differs
// =========================
void checkGantry() {
    int gState = digitalRead(gantryPin);

    if (gState == HIGH && !gantryFlag) {
        int gantryId = detectGantryId();

        if (gantryId != 0) {
            gantryFlag = true;
            currentGantryId = gantryId;
            gantryCount++;

            // Approximate loop count for reporting
            // Sequence is 5 gantries before parking
            if (gantryCount >= 3) {
                loopCount = 1;
            }
            if (gantryCount >= 5) {
                loopCount = 2;
            }

            stopMotors();
            delay(GANTRY_STOP_DELAY);

            Serial.print("GANTRY_ID: ");
            Serial.println(currentGantryId);

            Serial.print("GANTRY_COUNT: ");
            Serial.println(gantryCount);

            Serial.print("LOOP_COUNT: ");
            Serial.println(loopCount);

            if (gantryCount == 5 && currentGantryId == 2) {
                parkingMode = true;
                sendStatus("PARKING_ARMED", "RIGHT_GANTRY_SECOND_EXIT");
            } else {
                if (currentGantryId == 1) {
                    sendStatus("GANTRY_STOP", "LEFT_GANTRY");
                } else if (currentGantryId == 2) {
                    sendStatus("GANTRY_STOP", "RIGHT_GANTRY");
                } else if (currentGantryId == 3) {
                    sendStatus("GANTRY_STOP", "CENTER_GANTRY");
                } else {
                    sendStatus("GANTRY_STOP", "UNKNOWN_GANTRY");
                }
            }
        }
    }

    if (gState == LOW) {
        gantryFlag = false;
    }
}

// returns 1, 2, 3 or 0 if unknown
int detectGantryId() {
    unsigned long d = pulseIn(gantryPin, HIGH, 5000);

    if (d > 500 && d < 1500) {
        return 1;   // left
    } else if (d >= 1500 && d < 2500) {
        return 2;   // right
    } else if (d >= 2500 && d < 3500) {
        return 3;   // center/top
    }

    return 0;
}

// =========================
// INITIAL EXIT FROM PARKING
// Clockwise: move out and bias right to join main track
// Tune timings on hardware
// =========================
void handleInitialExit() {
    unsigned long now = millis();

    if (exitStep == 0) {
        forward();
        if (now - exitStepTime >= EXIT_FORWARD_TIME) {
            stopMotors();
            exitStep = 1;
            exitStepTime = now;
        }
    }
    else if (exitStep == 1) {
        turnRight();
        if (now - exitStepTime >= EXIT_RIGHT_TIME) {
            stopMotors();
            exitStep = 2;
            exitStepTime = now;
            sendStatus("ON_MAIN_TRACK", "MAIN_TRACK");
        }
    }
    else {
        exitParkingMode = false;
    }
}

// =========================
// MAIN TRACK FOLLOWING
// Assumes current sensor polarity:
// 1,1 -> follow forward
// 1,0 -> turn left
// 0,1 -> turn right
// 0,0 -> thick line / crossing -> continue
// =========================
void handleMainTrack(int left, int right) {
    if (left == 1 && right == 1) {
        forward();
    }
    else if (left == 1 && right == 0) {
        turnLeft();
    }
    else if (left == 0 && right == 1) {
        turnRight();
    }
    else {
        forward();
    }
}

// =========================
// PARKING HANDLER
// Parking is armed only after gantryCount == 5 at right gantry second time.
// Then use the next confirmed marker region to move into parking.
// For clockwise motion, inward turn is typically right.
// If your actual buggy needs left instead, swap turnRight() to turnLeft().
// =========================
void handleParking(int left, int right) {
    unsigned long now = millis();

    if (parkingStep == 0) {
        // Keep following until parking entry marker appears
        if (left == 1 && right == 1) {
            forward();
        }
        else if (left == 1 && right == 0) {
            turnLeft();
        }
        else if (left == 0 && right == 1) {
            turnRight();
        }
        else if (left == 0 && right == 0) {
            delay(40);
            if (digitalRead(LS) == 0 && digitalRead(RS) == 0) {
                stopMotors();
                sendStatus("PARKING_ENTRY_DETECTED", "ENTERING_PARKING");
                parkingStep = 1;
                parkingStepTime = millis();
            } else {
                forward();
            }
        }
    }
    else if (parkingStep == 1) {
        // move slightly ahead before entering parking lane
        forward();
        if (now - parkingStepTime >= PARK_FORWARD_1) {
            stopMotors();
            parkingStep = 2;
            parkingStepTime = now;
        }
    }
    else if (parkingStep == 2) {
        // clockwise inward turn likely right; tune on track
        turnRight();
        if (now - parkingStepTime >= PARK_RIGHT_1) {
            stopMotors();
            parkingStep = 3;
            parkingStepTime = now;
        }
    }
    else if (parkingStep == 3) {
        forward();
        if (now - parkingStepTime >= PARK_FORWARD_2) {
            stopMotors();
            parkingStep = 4;
            parked = true;
            sendStatus("PARKED", "PARKING_BAY");
        }
    }
}

// =========================
// ULTRASONIC
// =========================
int getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 20000);

    if (duration == 0) {
        return 0;
    }

    int dist = duration * 0.034 / 2;
    return dist;
}

// =========================
// STATUS MESSAGES
// =========================
void sendStatus(const char* buggyState, const char* trackState) {
    Serial.print("BUGGY_STATE: ");
    Serial.print(buggyState);
    Serial.print(" | TRACK_STATE: ");
    Serial.print(trackState);
    Serial.print(" | LOOP_COUNT: ");
    Serial.print(loopCount);
    Serial.print(" | GANTRY_COUNT: ");
    Serial.print(gantryCount);
    Serial.print(" | CURRENT_GANTRY_ID: ");
    Serial.println(currentGantryId);
}

// =========================
// MOTOR CONTROL
// =========================
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
