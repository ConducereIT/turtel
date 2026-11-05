// ============================================================================
//  ESP32 robot: CRSF receiver (AlfredoCRSF) + A4950 dual H-bridge drive
//  + brushless ESC with ramp-up, kill switch and arm switch.
//  Requires ESP32 Arduino core 3.x (ledcAttach / ledcWrite by pin).
// ============================================================================
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// ---------- Pins ----------
#define PIN_RX    16   // ESP32 RX  <- receiver TX
#define PIN_TX    17   // ESP32 TX  -> receiver RX
#define PIN_AIN1  22   // Motor A (left)
#define PIN_AIN2  23
#define PIN_BIN1  2    // Motor B (right)  (was the LED pin, now used by the H-bridge)
#define PIN_BIN2  4
#define PIN_ESC   25

// ---------- CRSF channels (1-indexed: 1..16) ----------
#define throttleChannel   2
#define steerChannel      4
#define directionChannel  7
#define armDisarmChannel  9
#define killSwitchChannel 5

// ---------- Switches (channel values are ~1000..2000 us) ----------
#define SWITCH_THRESHOLD 1500
#define KILL_WHEN_HIGH   true    // true: kill engaged when CH5 > threshold (flip if your switch is the other way)

// ---------- Drive (H-bridge) ----------
#define STICK_CENTER        1500
#define STICK_DEADBAND      30      // +/- us around center treated as zero
#define STICK_RANGE         500     // center -> end of stick travel
#define DRIVE_PWM_FREQ      20000   // 20 kHz: above audible range
#define DRIVE_PWM_BITS      8
#define DRIVE_MAX           255
#define BRAKE_WHEN_STOPPED  false   // true: active brake at neutral, false: coast

// ---------- ESC ----------
#define ESC_FREQ       50
#define ESC_BITS       16
#define ESC_PERIOD_US  20000

#define US_OFF         1000   // motor stopped
#define US_PROBE       1100   // phase 1: low start (tune per motor/ESC)
#define US_PHASE2_END  1900   // end of aggressive acceleration
#define US_MAX         2000   // final ceiling

#define T_PHASE1   600        // 0   - 0.6 s: gentle start
#define T_PHASE2   1700       // 0.6 - 1.7 s: aggressive acceleration
#define T_PHASE3   2000       // 1.7 - 2.0 s: finish at max
#define T_ESC_ARM  3000       // ESC arming at power-up (ms)

// ---------- Debug output ----------
#define PRINT_INTERVAL     100
#define LINK_MSG_INTERVAL  500

// ============================================================================
struct RcInputs {
  int  throttle, steer, direction, arm, kill;   // raw channel values (us)
  bool killActive;                              // kill switch engaged
  bool armOn;                                   // weapon arm switch on
  bool inverted;                                // drive inverted (robot flipped)
};

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// Drive state
int driveLeft = 0, driveRight = 0;

// Weapon / ESC state
int  escUs = US_OFF;
bool rampActive = false;
unsigned long rampStart = 0;
bool weaponReady = false;   // arm switch must be seen OFF once before the weapon may spin

// ============================================================================
//  Setup / loop
// ============================================================================
void setup() {
  Serial.begin(115200);

  setupDrive();
  setupEsc();          // blocks T_ESC_ARM ms with the ESC at 1000 us

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
  Serial.println("Waiting for CRSF link...");
}

void loop() {
  crsf.update();                       // call as often as possible

  if (!crsf.isLinkUp()) {              // fail-safe: no link -> everything off
    enterSafeState();
    printLinkDown();
    return;
  }

  RcInputs in = readInputs();

  if (in.killActive) {
    enterSafeState();
  } else {
    updateDrive(in);
    updateWeapon(in);
  }

  printStatus(in);
}

// ============================================================================
//  Inputs
// ============================================================================
bool isHigh(int us) { return us > SWITCH_THRESHOLD; }

RcInputs readInputs() {
  RcInputs in;
  in.throttle  = crsf.getChannel(throttleChannel);
  in.steer     = crsf.getChannel(steerChannel);
  in.direction = crsf.getChannel(directionChannel);
  in.arm       = crsf.getChannel(armDisarmChannel);
  in.kill      = crsf.getChannel(killSwitchChannel);

  in.killActive = (isHigh(in.kill) == KILL_WHEN_HIGH);
  in.armOn      = isHigh(in.arm);
  in.inverted   = isHigh(in.direction);
  return in;
}

// Stick (us) -> -DRIVE_MAX..DRIVE_MAX with deadband around center
int stickToSpeed(int us) {
  int offset = us - STICK_CENTER;
  int mag = abs(offset);
  if (mag <= STICK_DEADBAND) return 0;
  int speed = map(mag, STICK_DEADBAND, STICK_RANGE, 0, DRIVE_MAX);
  speed = constrain(speed, 0, DRIVE_MAX);
  return offset > 0 ? speed : -speed;
}

// ============================================================================
//  Safety
// ============================================================================
void enterSafeState() {
  stopDrive();
  stopWeapon();
  weaponReady = false;     // after kill / link loss the arm switch must be cycled
}

// ============================================================================
//  Drive (A4950 H-bridge, PWM directly on IN pins)
// ============================================================================
void setupDrive() {
  ledcAttach(PIN_AIN1, DRIVE_PWM_FREQ, DRIVE_PWM_BITS);
  ledcAttach(PIN_AIN2, DRIVE_PWM_FREQ, DRIVE_PWM_BITS);
  ledcAttach(PIN_BIN1, DRIVE_PWM_FREQ, DRIVE_PWM_BITS);
  ledcAttach(PIN_BIN2, DRIVE_PWM_FREQ, DRIVE_PWM_BITS);
  stopDrive();
}

// v > 0 forward, v < 0 reverse, 0 coast
void setMotor(int in1, int in2, int v) {
  v = constrain(v, -DRIVE_MAX, DRIVE_MAX);
  if (v > 0) {
    ledcWrite(in1, v);
    ledcWrite(in2, 0);
  } else if (v < 0) {
    ledcWrite(in1, 0);
    ledcWrite(in2, -v);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

void setDrive(int left, int right) {
  driveLeft  = left;
  driveRight = right;
  setMotor(PIN_AIN1, PIN_AIN2, left);
  setMotor(PIN_BIN1, PIN_BIN2, right);
}

void stopDrive() { setDrive(0, 0); }     // coast

void brakeDrive() {                      // both inputs fully on = active brake
  driveLeft = driveRight = 0;
  ledcWrite(PIN_AIN1, DRIVE_MAX);        // core 3.x treats max duty as 100 %
  ledcWrite(PIN_AIN2, DRIVE_MAX);
  ledcWrite(PIN_BIN1, DRIVE_MAX);
  ledcWrite(PIN_BIN2, DRIVE_MAX);
}

// Arcade mix: throttle + steer -> left/right
void updateDrive(const RcInputs& in) {
  int t = stickToSpeed(in.throttle);
  int s = stickToSpeed(in.steer);

  if (in.inverted) t = -t;   // robot upside down: forward stays forward, steering already correct

  int left  = constrain(t + s, -DRIVE_MAX, DRIVE_MAX);
  int right = constrain(t - s, -DRIVE_MAX, DRIVE_MAX);

  if (left == 0 && right == 0 && BRAKE_WHEN_STOPPED) brakeDrive();
  else                                               setDrive(left, right);
}

// ============================================================================
//  Weapon (brushless ESC with ramp-up)
// ============================================================================
void setupEsc() {
  ledcAttach(PIN_ESC, ESC_FREQ, ESC_BITS);
  writeEsc(US_OFF);
  Serial.println("Arming ESC (1000 us, 3 s)... DO NOT touch the motor");
  delay(T_ESC_ARM);
}

void writeEsc(int us) {
  us = constrain(us, US_OFF, US_MAX);
  escUs = us;
  // 50 Hz, 16 bit: 20000 us period -> 65535
  ledcWrite(PIN_ESC, (uint32_t)us * 65535UL / ESC_PERIOD_US);
}

int rampValue(unsigned long t) {         // t = ms since ramp start
  if (t < T_PHASE1) return US_PROBE;
  if (t < T_PHASE2) return map(t, T_PHASE1, T_PHASE2, US_PROBE, US_PHASE2_END);
  if (t < T_PHASE3) return map(t, T_PHASE2, T_PHASE3, US_PHASE2_END, US_MAX);
  return US_MAX;
}

void stopWeapon() {
  rampActive = false;
  writeEsc(US_OFF);
}

void updateWeapon(const RcInputs& in) {
  if (!in.armOn) {          // switch off: stop, and allow the next arming
    weaponReady = true;
    stopWeapon();
    return;
  }

  if (!weaponReady) {       // switch was already on at link-up / after kill
    stopWeapon();
    return;
  }

  if (!rampActive) {        // just armed -> start ramp
    rampActive = true;
    rampStart = millis();
  }
  writeEsc(rampValue(millis() - rampStart));
}

// ============================================================================
//  Debug output (throttled, never blocks control)
// ============================================================================
void printLinkDown() {
  static unsigned long lastMsg = 0;
  if (millis() - lastMsg < LINK_MSG_INTERVAL) return;
  lastMsg = millis();
  Serial.println("Link down -> SAFE (drive stopped, ESC 1000 us)");
}

const char* weaponStateText(const RcInputs& in) {
  if (in.killActive)          return "KILLED";
  if (!in.armOn)              return "DISARMED";
  if (!weaponReady)           return "CYCLE ARM SW";
  return rampActive ? "SPINNING" : "ARMED";
}

void printStatus(const RcInputs& in) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < PRINT_INTERVAL) return;
  lastPrint = millis();

  Serial.print("Thr: ");      Serial.print(in.throttle);
  Serial.print("\tSteer: ");  Serial.print(in.steer);
  Serial.print("\tDir: ");    Serial.print(in.inverted ? "INV" : "NORM");
  Serial.print("\tL/R: ");    Serial.print(driveLeft);
  Serial.print("/");          Serial.print(driveRight);
  Serial.print("\tWeapon: "); Serial.print(weaponStateText(in));
  Serial.print("\tESC: ");    Serial.print(escUs);
  Serial.print(" us");
  if (rampActive) {
    Serial.print(" (ramp ");
    Serial.print(millis() - rampStart);
    Serial.print(" ms)");
  }
  Serial.println();
}
