#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <TimeAlarms.h>

// ─── Your location (Udumalpet area, Tamil Nadu) ──────────────
const float LAT =  10.57997;
const float LON =  77.24842;
const float TZ  =  5.5;        // IST = UTC+5:30

// ─── Relay ───────────────────────────────────────────────────
const int RELAY = 7;

// ─── Config ──────────────────────────────────────────────────
const int MORNING_BEFORE_SUNRISE_MIN = 60;
const int MORNING_DURATION_MIN       = 60;
const int EVENING_DURATION_MIN       = 240;

// ─── State ───────────────────────────────────────────────────
bool   relayMorningActive = false;
bool   relayEveningActive = false;
time_t relayOnTimeMorning = 0;
time_t relayOnTimeEvening = 0;

long morningOnSec = 0;   // seconds since midnight for morning ON
long eveningOnSec = 0;   // seconds since midnight for evening ON
int lastCalcDay  = -1;

AlarmID_t morningAlarmID = dtINVALID_ALARM_ID;
AlarmID_t eveningAlarmID = dtINVALID_ALARM_ID;

RTC_DS3231 RTC;

// ─── TimeLib sync bridge ─────────────────────────────────────
time_t getTime() {
  return RTC.now().unixtime();
}

// ════════════════════════════════════════════════════════════
//  SOLAR CALCULATOR  (Jean Meeus simplified)
//  type: 0 = sunrise, 1 = civil dusk (sun 6° below horizon)
//  Returns local time as minutes since midnight
// ════════════════════════════════════════════════════════════
float calcSunEvent(int yr, int mo, int dy, float lat, float lon, float tz, int type) {
  // Day offset from J2000.0 epoch (integer part exact, no float precision loss)
  int a = (14 - mo) / 12;
  int y = yr + 4800 - a;
  int mm = mo + 12 * a - 3;
  unsigned long JDN = (unsigned long)dy
                    + (unsigned long)((153 * mm + 2) / 5)
                    + (unsigned long)365 * y
                    + (unsigned long)(y / 4)
                    - (unsigned long)(y / 100)
                    + (unsigned long)(y / 400)
                    - 32045UL;

  // n = days since J2000.0 noon — keep as float offset (small number ~9000)
  float n = (float)((long)JDN - 2451545L) - 0.5;

  // Mean longitude and mean anomaly (degrees)
  float L = fmod(280.460 + 0.9856474 * n, 360.0);
  float g = fmod(357.528 + 0.9856003 * n, 360.0);
  if (L < 0) L += 360.0;
  if (g < 0) g += 360.0;

  float gr = g * DEG_TO_RAD;

  // Ecliptic longitude
  float lambda = L + 1.915 * sin(gr) + 0.020 * sin(2.0 * gr);
  float lr     = lambda * DEG_TO_RAD;

  // Declination
  float sinDec = sin(23.439 * DEG_TO_RAD) * sin(lr);
  float dec    = asin(sinDec);

  // Equation of time (minutes) → convert to hours
  float LL  = fmod(L - (lambda - 360.0 * floor(lambda / 360.0)), 360.0);
  float EoT = LL / 15.0;   // hours

  // Solar noon in UTC hours
  float solarNoonUTC = 12.0 - lon / 15.0 - EoT;

  // Hour angle for sunrise/sunset or civil dusk
  // type 0 = sunrise/sunset  h0 = -0.8333°
  // type 1 = civil dusk      h0 = -6.0°
  float h0   = (type == 0) ? -0.8333 : -6.0;
  float latR = lat * DEG_TO_RAD;

  float cosH = (sin(h0 * DEG_TO_RAD) - sin(latR) * sinDec)
             / (cos(latR) * cos(dec));

  if (cosH < -1.0 || cosH > 1.0) return -1;  // midnight sun / polar night

  float H = acos(cosH) * RAD_TO_DEG;  // hour angle in degrees

  // Sunrise = solar noon - H/15 hours
  // Sunset  = solar noon + H/15 hours
  float utcH;
  if (type == 0) {
    utcH = solarNoonUTC - H / 15.0;  // sunrise
  } else {
    utcH = solarNoonUTC + H / 15.0;  // civil dusk
  }

  // Normalize to 0–24
  while (utcH <   0.0) utcH += 24.0;
  while (utcH >= 24.0) utcH -= 24.0;

  float localH = utcH + tz;
  while (localH >= 24.0) localH -= 24.0;
  while (localH <   0.0) localH += 24.0;

  return localH * 60.0;  // minutes since midnight
}

// ─── Print minutes-since-midnight as HH:MM ───────────────────
void printMinutes(int totalMin) {
  int h = totalMin / 60;
  int m = totalMin % 60;
  Serial.print(h);
  Serial.print(":");
  if (m < 10) Serial.print("0");
  Serial.println(m);
}

// ADD this temporary test in setup() right after Serial.begin(9600):
void testSolar() {
  Serial.println("=== SOLAR DEBUG ===");
  int yr = 2025, mo = 6, dy = 13;

  int a = (14 - mo) / 12;
  int y = yr + 4800 - a;
  int mm = mo + 12 * a - 3;
  unsigned long JDN = (unsigned long)dy
                    + (unsigned long)((153 * mm + 2) / 5)
                    + (unsigned long)365 * y
                    + (unsigned long)(y / 4)
                    - (unsigned long)(y / 100)
                    + (unsigned long)(y / 400)
                    - 32045UL;

  Serial.print("JDN (integer) = "); Serial.println(JDN);   // expect 2460839

  float n = (float)((long)JDN - 2451545L) - 0.5;
  Serial.print("n = "); Serial.println(n, 2);               // expect ~9293.5

  float L = fmod(280.460 + 0.9856474 * n, 360.0);
  float g = fmod(357.528 + 0.9856003 * n, 360.0);
  if (L < 0) L += 360.0;
  if (g < 0) g += 360.0;
  Serial.print("L = "); Serial.println(L, 3);
  Serial.print("g = "); Serial.println(g, 3);

  float gr     = g * DEG_TO_RAD;
  float lambda = L + 1.915 * sin(gr) + 0.020 * sin(2.0 * gr);
  float lr     = lambda * DEG_TO_RAD;
  float sinDec = sin(23.439 * DEG_TO_RAD) * sin(lr);
  float dec    = asin(sinDec);
  Serial.print("lambda = "); Serial.println(lambda, 3);
  Serial.print("dec (deg) = "); Serial.println(dec * RAD_TO_DEG, 3);  // expect ~23°

  float LL         = fmod(L - (lambda - 360.0 * floor(lambda / 360.0)), 360.0);
  float EoT        = LL / 15.0;
  float solarNoon  = 12.0 - 77.24842 / 15.0 + (-EoT);
  Serial.print("EoT (hrs) = "); Serial.println(EoT, 4);
  Serial.print("Solar noon UTC = "); Serial.println(solarNoon, 4);  // expect ~6.8

  float latR = 10.57997 * DEG_TO_RAD;
  float cosH = (sin(-0.8333 * DEG_TO_RAD) - sin(latR) * sinDec) / (cos(latR) * cos(dec));
  Serial.print("cosH = "); Serial.println(cosH, 6);   // expect between -1 and 1

  float H      = acos(cosH) * RAD_TO_DEG;
  float utcRise = solarNoon - H / 15.0;
  float utcSet  = solarNoon + H / 15.0;
  Serial.print("H (deg) = "); Serial.println(H, 3);   // expect ~90°
  Serial.print("UTC sunrise = "); Serial.println(utcRise, 4);  // expect ~0.5
  Serial.print("UTC sunset  = "); Serial.println(utcSet,  4);  // expect ~13.1
  Serial.print("IST sunrise = "); Serial.println(utcRise + 5.5, 4);  // expect ~6.0
  Serial.print("IST sunset  = "); Serial.println(utcSet  + 5.5, 4);  // expect ~18.6
  Serial.println("===================");
}

// ─── Recalculate alarm times for today ───────────────────────
void calcTodayAlarms() {
  DateTime dt = RTC.now();   // read directly from RTC — no TimeLib dependency
  int yr = dt.year();
  int mo = dt.month();
  int dy = dt.day();

  float sunriseMins  = calcSunEvent(yr, mo, dy, LAT, LON, TZ, 0);
  float civilDuskMin = calcSunEvent(yr, mo, dy, LAT, LON, TZ, 1);

  float morningMins  = sunriseMins - MORNING_BEFORE_SUNRISE_MIN;
  if (morningMins < 0) morningMins += 1440;

  morningOnSec = (long)(morningMins  * 60);
  eveningOnSec = (long)(civilDuskMin * 60);

  Serial.print("Sunrise      : "); printMinutes((int)sunriseMins);
  Serial.print("Civil dusk   : "); printMinutes((int)civilDuskMin);
  Serial.print("Morning ON   : "); printMinutes((int)morningMins);
  Serial.print("Evening ON   : "); printMinutes((int)civilDuskMin);

  lastCalcDay = dy;

  // Free previous alarms
  if (morningAlarmID != dtINVALID_ALARM_ID) {
    Alarm.free(morningAlarmID);
    morningAlarmID = dtINVALID_ALARM_ID;
  }
  if (eveningAlarmID != dtINVALID_ALARM_ID) {
    Alarm.free(eveningAlarmID);
    eveningAlarmID = dtINVALID_ALARM_ID;
  }

  // Register new alarms
  int mH = (int)(morningOnSec / 3600);
  int mM = (int)((morningOnSec % 3600) / 60);
  int eH = (int)(eveningOnSec / 3600);
  int eM = (int)((eveningOnSec % 3600) / 60);

  morningAlarmID = Alarm.alarmRepeat(mH, mM, 0, MorningAlarm);
  eveningAlarmID = Alarm.alarmRepeat(eH, eM, 0, EveningAlarm);

  Serial.print("Alarms set → Morning ");
  Serial.print(mH); Serial.print(":"); if (mM < 10) Serial.print("0"); Serial.print(mM);
  Serial.print("  Evening ");
  Serial.print(eH); Serial.print(":"); if (eM < 10) Serial.print("0"); Serial.println(eM);
}

// ════════════════════════════════════════════════════════════
//  POWER CUT RECOVERY
//  Reads time directly from RTC, calculates window using
//  morningOnSec / eveningOnSec set by calcTodayAlarms()
//  No dependency on TimeLib previousMidnight()
// ════════════════════════════════════════════════════════════
void checkRelayRecovery() {
  DateTime dt = RTC.now();   // direct RTC read — reliable immediately on boot

  // Current time as seconds since midnight
  long nowSec = (long)dt.hour() * 3600L + dt.minute() * 60L + dt.second();

  long morningOffSec = morningOnSec + MORNING_DURATION_MIN * 60L;
  long eveningOffSec = eveningOnSec + EVENING_DURATION_MIN * 60L;

  Serial.print("Boot time (sec since midnight): ");
  Serial.println(nowSec);
  Serial.print("Morning window: ");
  Serial.print(morningOnSec); Serial.print(" → "); Serial.println(morningOffSec);
  Serial.print("Evening window: ");
  Serial.print(eveningOnSec); Serial.print(" → "); Serial.println(eveningOffSec);

  // Morning recovery
  if (nowSec >= morningOnSec && nowSec < morningOffSec) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    // Set relayOnTimeMorning so remaining duration is correct
    // now() is synced at this point since setSyncProvider was called before
    relayOnTimeMorning = now() - (nowSec - morningOnSec);
    Serial.println(">>> RECOVERED: Morning relay ON after power cut");
    Serial.print("Time already elapsed (min): ");
    Serial.println((nowSec - morningOnSec) / 60);
    Serial.print("Remaining (min): ");
    Serial.println((morningOffSec - nowSec) / 60);
  }

  // Evening recovery
  if (nowSec >= eveningOnSec && nowSec < eveningOffSec) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now() - (nowSec - eveningOnSec);
    Serial.println(">>> RECOVERED: Evening relay ON after power cut");
    Serial.print("Time already elapsed (min): ");
    Serial.println((nowSec - eveningOnSec) / 60);
    Serial.print("Remaining (min): ");
    Serial.println((eveningOffSec - nowSec) / 60);
  }

  if (!relayMorningActive && !relayEveningActive) {
    Serial.println("Boot outside any ON window — relay stays OFF");
  }
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);   // relay OFF initially

  Serial.begin(9600);
  while (!Serial);
  testSolar();

  if (!RTC.begin()) {
    Serial.println("RTC not found! Check wiring.");
    while (1);
  }
  Serial.println("RTC OK");

  setSyncProvider(getTime);
  setSyncInterval(300);

  // Wait until TimeLib is synced from RTC
  int retries = 0;
  while (timeStatus() != timeSet && retries < 10) {
    delay(200);
    retries++;
  }
  if (timeStatus() != timeSet) {
    Serial.println("WARNING: TimeLib sync failed, using RTC directly");
  } else {
    Serial.println("TimeLib synced from RTC");
  }

  calcTodayAlarms();      // sets morningOnSec + eveningOnSec FIRST
  checkRelayRecovery();   // then recovery uses those values
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  Alarm.delay(1000);
  digitalClockDisplay();

  // Recalculate at midnight for next day
  DateTime dt = RTC.now();
  if (dt.day() != lastCalcDay) {
    Serial.println("New day — recalculating solar times...");
    calcTodayAlarms();
  }

  // Morning OFF
  if (relayMorningActive && now() - relayOnTimeMorning >= MORNING_DURATION_MIN * 60L) {
    digitalWrite(RELAY, HIGH);
    relayMorningActive = false;
    Serial.println("Morning OFF");
  }

  // Evening OFF
  if (relayEveningActive && now() - relayOnTimeEvening >= EVENING_DURATION_MIN * 60L) {
    digitalWrite(RELAY, HIGH);
    relayEveningActive = false;
    Serial.println("Evening OFF");
  }
}

// ─── Alarm callbacks ─────────────────────────────────────────
void MorningAlarm() {
  Serial.println("Morning ON");
  digitalWrite(RELAY, LOW);
  relayMorningActive = true;
  relayOnTimeMorning = now();
}

void EveningAlarm() {
  Serial.println("Evening ON");
  digitalWrite(RELAY, LOW);
  relayEveningActive = true;
  relayOnTimeEvening = now();
}

// ─── Clock display ───────────────────────────────────────────
void digitalClockDisplay() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println();
}

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10) Serial.print("0");
  Serial.print(digits);
}
