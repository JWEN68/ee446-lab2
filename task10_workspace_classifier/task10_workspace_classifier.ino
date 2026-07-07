/*
 * EE 446 Lab 2 — Task 10
 * Smart Workspace Situation Classifier
 *
 * Reads mic, light, motion, and proximity. Turns each into a 0/1 flag,
 * then picks one of four situation labels using simple rules.
 */

#include <PDM.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>

// ---- Microphone buffer (filled by interrupt) ----
short   sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

// ---- Thresholds ----
const int   TH_SOUND  = 150;    // mic level (quiet ~5, speech 400+)
const int   TH_DARK   = 40;     // clear channel (bright ~100, shaded ~20)
const float TH_MOTION = 0.15f;  // combined accel + gyro score
const int   TH_NEAR   = 100;    // proximity (0=touch, 255=far)

// ---- Timing ----
const unsigned long UPDATE_MS = 500;   // print every 500 ms
unsigned long lastUpdate = 0;

// ---- Latest readings ----
int   micLevel    = 0;
int   clearLevel  = 0;
int   proxLevel   = 255;
float motionScore = 0.0f;

// Average absolute mic amplitude (loud = big number)
int readMicLevel() {
  if (samplesRead > 0) {
    long sum = 0;
    for (int i = 0; i < samplesRead; i++) sum += abs(sampleBuffer[i]);
    int level = sum / samplesRead;
    samplesRead = 0;
    return level;
  }
  return micLevel;   // no new data, keep last value
}

// Motion score: how much the accelerometer differs from 1g + gyro magnitude
float computeMotion() {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  float aMag = sqrt(ax*ax + ay*ay + az*az);
  float aDev = fabs(aMag - 1.0f);              // 0 when still, big when moved
  float gMag = sqrt(gx*gx + gy*gy + gz*gz) / 200.0f;   // scale down deg/s

  return aDev + gMag;
}

// Pick the situation label from the four flags
const char* classify(bool sound, bool dark, bool moving, bool near_) {
  // Try to match one of the four required situations exactly
  if ( sound &&  near_ && moving && !dark)   return "NOISY_BRIGHT_MOVING_NEAR";
  if ( sound && !near_ && !moving && !dark)  return "NOISY_BRIGHT_STEADY_FAR";
  if (!sound &&  near_ && !moving &&  dark)  return "QUIET_DARK_STEADY_NEAR";
  if (!sound && !near_ && !moving && !dark)  return "QUIET_BRIGHT_STEADY_FAR";

  // If flags don't fit any exactly, pick the label with the most matching flags
  const bool targets[4][4] = {
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

void setup() {
  Serial.begin(115200);
  delay(1500);

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) { Serial.println("PDM init failed."); while (1); }
  if (!APDS.begin())        { Serial.println("APDS init failed."); while (1); }
  if (!IMU.begin())         { Serial.println("IMU init failed.");  while (1); }

  Serial.println("Smart Workspace Classifier started");
}

void loop() {
  if (millis() - lastUpdate < UPDATE_MS) return;
  lastUpdate = millis();

  // Read all four modalities
  micLevel    = readMicLevel();
  motionScore = computeMotion();

  if (APDS.colorAvailable())     { int r,g,b,c; APDS.readColor(r,g,b,c); clearLevel = c; }
  if (APDS.proximityAvailable()) { proxLevel = APDS.readProximity(); }

  // Turn readings into 0/1 flags
  bool sound  = (micLevel    > TH_SOUND);
  bool dark   = (clearLevel  < TH_DARK);
  bool moving = (motionScore > TH_MOTION);
  bool near_  = (proxLevel   < TH_NEAR);

  const char* label = classify(sound, dark, moving, near_);

  // Print the three required lines
  Serial.print("raw,mic=");    Serial.print(micLevel);
  Serial.print(",clear=");     Serial.print(clearLevel);
  Serial.print(",motion=");    Serial.print(motionScore, 3);
  Serial.print(",prox=");      Serial.println(proxLevel);

  Serial.print("flags,sound="); Serial.print(sound  ? 1 : 0);
  Serial.print(",dark=");       Serial.print(dark   ? 1 : 0);
  Serial.print(",moving=");     Serial.print(moving ? 1 : 0);
  Serial.print(",near=");       Serial.println(near_ ? 1 : 0);

  Serial.print("state,");       Serial.println(label);
  Serial.println();
}
