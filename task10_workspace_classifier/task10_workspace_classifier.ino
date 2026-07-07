/*
 * EE 446 TinyML — Lab 2, Task 10
 * Smart Workspace Situation Classifier
 *
 * Reads four onboard sensing modalities on the Arduino Nano 33 BLE Sense Rev2,
 * thresholds each into a binary flag, and combines them with rule-based logic
 * to print one of four required situation labels every update cycle.
 *
 * Sensors used:
 *   - PDM microphone      -> audio activity level
 *   - APDS9960 (clear ch) -> ambient brightness
 *   - BMI270 IMU          -> physical motion (accel deviation + gyro magnitude)
 *   - APDS9960 (prox ch)  -> user presence near the board
 *
 * Output format (three lines per update cycle):
 *   raw,mic=<v>,clear=<v>,motion=<v>,prox=<v>
 *   flags,sound=<0/1>,dark=<0/1>,moving=<0/1>,near=<0/1>
 *   state,<FINAL_LABEL>
 */

#include <PDM.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>

// ---------------- Microphone (PDM) plumbing ----------------
short   sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

// ---------------- Thresholds ----------------
// These are chosen empirically from Tasks 5, 6A/B, 9A, 9C observations.
// Justification is written up in the report — see /docs section of the repo.
const int   TH_SOUND  = 150;    // mic average abs. amplitude (quiet ~5, speech ~500+)
const int   TH_DARK   = 40;     // APDS clear channel (room ~100, shaded ~20)
const float TH_MOTION = 0.15f;  // motion score (see computeMotion below)
const int   TH_NEAR   = 100;    // proximity (0=touch, 255=far). <100 => hand within ~10 cm

// ---------------- Timing ----------------
const unsigned long UPDATE_MS = 500;   // print at 2 Hz
unsigned long lastUpdate = 0;

// ---------------- Per-cycle sensor state ----------------
int   micLevel    = 0;
int   clearLevel  = 0;
int   proxLevel   = 255;
float motionScore = 0.0f;

// ---------------- Helpers ----------------
int readMicLevel() {
  // Return latest average absolute amplitude, or previous value if no new batch.
  if (samplesRead > 0) {
    long sum = 0;
    for (int i = 0; i < samplesRead; i++) sum += abs(sampleBuffer[i]);
    int level = sum / samplesRead;
    samplesRead = 0;
    return level;
  }
  return micLevel;   // stale but fine; loop is fast enough
}

float computeMotion() {
  // Combine accelerometer deviation from 1g gravity with gyro magnitude.
  // Board still on a desk -> motion ~ 0.02 (sensor noise floor).
  // Board being picked up / rotated -> motion > 0.3.
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  // Magnitude of accel vector, minus 1g (gravity). Nonzero => real motion.
  float aMag = sqrt(ax*ax + ay*ay + az*az);
  float aDev = fabs(aMag - 1.0f);

  // Gyro magnitude in deg/s -> scale down so its contribution is comparable
  // to aDev. Divide by 200 so 200 deg/s ~ 1.0.
  float gMag = sqrt(gx*gx + gy*gy + gz*gz) / 200.0f;

  return aDev + gMag;
}

const char* classify(bool sound, bool dark, bool moving, bool near_) {
  // Priority-ordered match against the four required situations.
  // Exact matches first; ambiguous inputs fall back to nearest label.

  if ( sound &&  near_ && moving && !dark) return "NOISY_BRIGHT_MOVING_NEAR";
  if ( sound && !near_ && !moving && !dark) return "NOISY_BRIGHT_STEADY_FAR";
  if (!sound &&  near_ && !moving &&  dark) return "QUIET_DARK_STEADY_NEAR";
  if (!sound && !near_ && !moving && !dark) return "QUIET_BRIGHT_STEADY_FAR";

  // Fallback: pick the closest of the four by counting matching flags.
  // (Keeps output valid when reality doesn't fit a clean bucket.)
  const bool targets[4][4] = {
    // sound, dark, moving, near_
    { false, false, false, false }, // QUIET_BRIGHT_STEADY_FAR
    { true,  false, false, false }, // NOISY_BRIGHT_STEADY_FAR
    { false, true,  false, true  }, // QUIET_DARK_STEADY_NEAR
    { true,  false, true,  true  }  // NOISY_BRIGHT_MOVING_NEAR
  };
  const char* names[4] = {
    "QUIET_BRIGHT_STEADY_FAR",
    "NOISY_BRIGHT_STEADY_FAR",
    "QUIET_DARK_STEADY_NEAR",
    "NOISY_BRIGHT_MOVING_NEAR"
  };
  bool now[4] = { sound, dark, moving, near_ };

  int bestScore = -1;
  int bestIdx   = 0;
  for (int i = 0; i < 4; i++) {
    int score = 0;
    for (int j = 0; j < 4; j++) if (now[j] == targets[i][j]) score++;
    if (score > bestScore) { bestScore = score; bestIdx = i; }
  }
  return names[bestIdx];
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // PDM mic
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone.");
    while (1);
  }

  // APDS9960 (proximity + color/clear)
  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960.");
    while (1);
  }

  // IMU (accel + gyro)
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  Serial.println("Smart Workspace Classifier started");
}

void loop() {
  // Continuously refresh mic buffer via the PDM callback.
  // Update-and-print only every UPDATE_MS.
  if (millis() - lastUpdate < UPDATE_MS) return;
  lastUpdate = millis();

  // ---- Read all four modalities ----
  micLevel    = readMicLevel();
  motionScore = computeMotion();

  if (APDS.colorAvailable())     { int r,g,b,c; APDS.readColor(r,g,b,c); clearLevel = c; }
  if (APDS.proximityAvailable()) { proxLevel = APDS.readProximity(); }

  // ---- Threshold into flags ----
  bool sound  = (micLevel    > TH_SOUND);
  bool dark   = (clearLevel  < TH_DARK);
  bool moving = (motionScore > TH_MOTION);
  bool near_  = (proxLevel   < TH_NEAR);

  // ---- Classify ----
  const char* label = classify(sound, dark, moving, near_);

  // ---- Print in the required 3-line format ----
  Serial.print("raw,mic=");    Serial.print(micLevel);
  Serial.print(",clear=");     Serial.print(clearLevel);
  Serial.print(",motion=");    Serial.print(motionScore, 3);
  Serial.print(",prox=");      Serial.println(proxLevel);

  Serial.print("flags,sound="); Serial.print(sound  ? 1 : 0);
  Serial.print(",dark=");       Serial.print(dark   ? 1 : 0);
  Serial.print(",moving=");     Serial.print(moving ? 1 : 0);
  Serial.print(",near=");       Serial.println(near_ ? 1 : 0);

  Serial.print("state,");       Serial.println(label);
  Serial.println();   // blank line between cycles for readability
}
