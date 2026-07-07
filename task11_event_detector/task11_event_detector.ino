/*
 * EE 446 Lab 2 — Task 11
 * Environmental Event Detector
 *
 * Watches humidity, temperature, magnetic field, and light/color for
 * sudden changes from a rolling baseline. Prints one of four event labels.
 */

#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>

// ---- Thresholds (how big a change counts as an event) ----
const float TH_HUMID_JUMP    = 5.0f;    // %RH above baseline
const float TH_TEMP_RISE     = 0.4f;    // C above baseline
const float TH_MAG_SHIFT     = 30.0f;   // uT change in |B|
const int   TH_CLEAR_CHANGE  = 30;      // clear channel change
const float TH_COLOR_BALANCE = 0.15f;   // RGB ratio change

// ---- Timing ----
const unsigned long UPDATE_MS   = 500;    // print every 500 ms
const unsigned long COOLDOWN_MS = 3000;   // hold event label for 3 s
const float BASELINE_ALPHA      = 0.02f;  // how fast baseline follows drift

unsigned long lastUpdate    = 0;
unsigned long lastEventTime = 0;
const char*   lastEvent     = "BASELINE_NORMAL";

// ---- Baseline values (learned automatically) ----
bool  baselineReady = false;
float rhBase = 0, tempBase = 0, magBase = 0;
int   clearBase = 0;
float rRatioBase = 0, gRatioBase = 0, bRatioBase = 0;

// Read total magnetic field strength
float magMagnitude() {
  float mx = 0, my = 0, mz = 0;
  if (IMU.magneticFieldAvailable()) IMU.readMagneticField(mx, my, mz);
  return sqrt(mx*mx + my*my + mz*mz);
}

// Slowly update the baseline toward the current reading
void updateBaseline(float rh, float t, float mag,
                    int c, float rR, float gR, float bR) {
  rhBase     = (1 - BASELINE_ALPHA) * rhBase     + BASELINE_ALPHA * rh;
  tempBase   = (1 - BASELINE_ALPHA) * tempBase   + BASELINE_ALPHA * t;
  magBase    = (1 - BASELINE_ALPHA) * magBase    + BASELINE_ALPHA * mag;
  clearBase  = (int)((1 - BASELINE_ALPHA) * clearBase + BASELINE_ALPHA * c);
  rRatioBase = (1 - BASELINE_ALPHA) * rRatioBase + BASELINE_ALPHA * rR;
  gRatioBase = (1 - BASELINE_ALPHA) * gRatioBase + BASELINE_ALPHA * gR;
  bRatioBase = (1 - BASELINE_ALPHA) * bRatioBase + BASELINE_ALPHA * bR;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!HS300x.begin()) { Serial.println("HS300x init failed."); while (1); }
  if (!IMU.begin())    { Serial.println("IMU init failed.");    while (1); }
  if (!APDS.begin())   { Serial.println("APDS init failed.");   while (1); }

  Serial.println("Environmental Event Detector started");
}

void loop() {
  if (millis() - lastUpdate < UPDATE_MS) return;
  lastUpdate = millis();

  // Read all sensors
  float rh  = HS300x.readHumidity();
  float t   = HS300x.readTemperature();
  float mag = magMagnitude();

  int r = 0, g = 0, b = 0, c = 0;
  if (APDS.colorAvailable()) APDS.readColor(r, g, b, c);

  // Convert RGB to fractions so we can detect color shifts
  int rgbSum = r + g + b;
  float rR = rgbSum > 0 ? (float)r / rgbSum : 0;
  float gR = rgbSum > 0 ? (float)g / rgbSum : 0;
  float bR = rgbSum > 0 ? (float)b / rgbSum : 0;

  // First cycle: seed the baseline
  if (!baselineReady) {
    rhBase = rh; tempBase = t; magBase = mag; clearBase = c;
    rRatioBase = rR; gRatioBase = gR; bRatioBase = bR;
    baselineReady = true;
  }

  // Check each modality against the baseline
  bool humid_jump = (rh - rhBase)   > TH_HUMID_JUMP;
  bool temp_rise  = (t  - tempBase) > TH_TEMP_RISE;
  bool mag_shift  = fabs(mag - magBase) > TH_MAG_SHIFT;

  bool clear_shift = abs(c - clearBase) > TH_CLEAR_CHANGE;
  bool color_shift = (fabs(rR - rRatioBase) > TH_COLOR_BALANCE) ||
                     (fabs(gR - gRatioBase) > TH_COLOR_BALANCE) ||
                     (fabs(bR - bRatioBase) > TH_COLOR_BALANCE);
  bool light_or_color_change = clear_shift || color_shift;

  // Pick the event label (with cooldown to avoid spam)
  const char* label = "BASELINE_NORMAL";
  bool anyEvent = humid_jump || temp_rise || mag_shift || light_or_color_change;

  if (anyEvent) {
    if (millis() - lastEventTime > COOLDOWN_MS) {
      if (humid_jump || temp_rise)  label = "BREATH_OR_WARM_AIR_EVENT";
      else if (mag_shift)           label = "MAGNETIC_DISTURBANCE_EVENT";
      else                          label = "LIGHT_OR_COLOR_CHANGE_EVENT";
      lastEvent = label;
      lastEventTime = millis();
    } else {
      label = lastEvent;   // still cooling down, keep same label
    }
  }

  // Only update baseline when nothing is happening
  if (!anyEvent) {
    updateBaseline(rh, t, mag, c, rR, gR, bR);
  }

  // Print the three required lines
  Serial.print("raw,rh=");   Serial.print(rh, 2);
  Serial.print(",temp=");    Serial.print(t, 2);
  Serial.print(",mag=");     Serial.print(mag, 2);
  Serial.print(",r=");       Serial.print(r);
  Serial.print(",g=");       Serial.print(g);
  Serial.print(",b=");       Serial.print(b);
  Serial.print(",clear=");   Serial.println(c);

  Serial.print("flags,humid_jump=");        Serial.print(humid_jump ? 1 : 0);
  Serial.print(",temp_rise=");              Serial.print(temp_rise  ? 1 : 0);
  Serial.print(",mag_shift=");              Serial.print(mag_shift  ? 1 : 0);
  Serial.print(",light_or_color_change=");  Serial.println(light_or_color_change ? 1 : 0);

  Serial.print("event,"); Serial.println(label);
  Serial.println();
}
