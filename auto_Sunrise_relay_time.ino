#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <EEPROM.h>

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

// ─── EEPROM addresses (store day-of-month, 1-31; 0 = never run) ──
const int EEPROM_ADDR_MORNING_DAY = 0;
const int EEPROM_ADDR_EVENING_DAY = 1;

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

// ─── EEPROM helpers ───────────────────────────────────────────
void markMorningCompleted(int dayOfMonth) {
  EEPROM.update(EEPROM_ADDR_MORNING_DAY, dayOfMonth);
}
void markEveningCompleted(int dayOfMonth) {
  EEPROM.update(EEPROM_ADDR_EVENING_DAY, dayOfMonth);
}
int getMorningCompletedDay() {
  return EEPROM.read(EEPROM_ADDR_MORNING_DAY);
}
int getEveningCompletedDay() {
  return EEPROM.read(EEPROM_ADDR_EVENING_DAY);
}

// ════════════════════════════════════════════════════════════
//  SOLAR CALCULATOR  (Jean Meeus simplified)
//  type: 0 = sunrise, 1 = civil dusk (sun 6° below horizon)
//  Returns local time as minutes since midnight
// ════════════════════════════════════════════════════════════
float calcSunEvent(int yr, int mo, int dy, float lat, float lon, float tz, int type) {
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

  float n = (float)((long)JDN - 2451545L) - 0.5;

  float L = fmod(280.460 + 0.9856474 * n, 360.0);
  float g = fmod(357.528 + 0.9856003 * n, 360.0);
  if (L < 0) L += 360.0;
  if (g < 0) g += 360.0;

  float gr = g * DEG_TO_RAD;

  float lambda = L + 1.915 * sin(gr) + 0.020 * sin(2.0 * gr);
  float lr     = lambda * DEG_TO_RAD;

  float sinDec = sin(23.439 * DEG_TO_RAD) * sin(lr);
  float dec    = asin(sinDec);

  float LL  = fmod(L - (lambda - 360.0 * floor(lambda / 360.0)), 360.0);
  float EoT = LL / 15.0;

  float solarNoonUTC = 12.0 - lon / 15.0 - EoT;

  float h0   = (type == 0) ? -0.8333 : -6.0;
  float latR = lat * DEG_TO_RAD;

  float cosH = (sin(h0 * DEG_TO_RAD) - sin(latR) * sinDec)
             / (cos(latR) * cos(dec));

  if (cosH < -1.0 || cosH > 1.0) return -1;

  float H = acos(cosH) * RAD_TO_DEG;

  float utcH;
  if (type == 0) {
    utcH = solarNoonUTC - H / 15.0;
  } else {
    utcH = solarNoonUTC + H / 15.0;
  }

  while (utcH <   0.0) utcH += 24.0;
  while (utcH >= 24.0) utcH -= 24.0;

  float localH = utcH + tz;
  while (localH >= 24.0) localH -= 24.0;
  while (localH <   0.0) localH += 24.0;

  return localH * 60.0;
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

// ─── Recalculate alarm times for today ───────────────────────
void calcTodayAlarms() {
  DateTime dt = RTC.now();
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

  if (morningAlarmID != dtINVALID_ALARM_ID) {
    Alarm.free(morningAlarmID);
    morningAlarmID = dtINVALID_ALARM_ID;
  }
  if (eveningAlarmID != dtINVALID_ALARM_ID) {
    Alarm.free(eveningAlarmID);
    eveningAlarmID = dtINVALID_ALARM_ID;
  }

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
//  POWER CUT / RESET RECOVERY
//  1) If boot happens INSIDE a window → resume mid-way (existing logic)
//  2) If boot happens AFTER a window has fully closed AND that window
//     never completed today (per EEPROM) → CATCH UP: turn relay on
//     now and run it for the FULL configured duration.
// ════════════════════════════════════════════════════════════
void checkRelayRecovery() {
  DateTime dt = RTC.now();
  int today = dt.day();

  long nowSec = (long)dt.hour() * 3600L + dt.minute() * 60L + dt.second();

  long morningOffSec = morningOnSec + MORNING_DURATION_MIN * 60L;
  long eveningOffSec = eveningOnSec + EVENING_DURATION_MIN * 60L;

  Serial.print("Boot time (sec since midnight): ");
  Serial.println(nowSec);
  Serial.print("Morning window: ");
  Serial.print(morningOnSec); Serial.print(" → "); Serial.println(morningOffSec);
  Serial.print("Evening window: ");
  Serial.print(eveningOnSec); Serial.print(" → "); Serial.println(eveningOffSec);

  bool didSomething = false;

  // ── Morning: boot INSIDE window → resume ──────────────────
  if (nowSec >= morningOnSec && nowSec < morningOffSec) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now() - (nowSec - morningOnSec);
    Serial.println(">>> RECOVERED: Morning relay ON after power cut (resumed mid-window)");
    Serial.print("Time already elapsed (min): ");
    Serial.println((nowSec - morningOnSec) / 60);
    Serial.print("Remaining (min): ");
    Serial.println((morningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Morning: boot AFTER window closed AND never completed today → catch up ──
  else if (nowSec >= morningOffSec && getMorningCompletedDay() != today) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now();   // full fresh duration starts now
    Serial.println(">>> CATCH-UP: Morning window was missed entirely — running full duration now");
    didSomething = true;
  }

  // ── Evening: boot INSIDE window → resume ──────────────────
  if (nowSec >= eveningOnSec && nowSec < eveningOffSec) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now() - (nowSec - eveningOnSec);
    Serial.println(">>> RECOVERED: Evening relay ON after power cut (resumed mid-window)");
    Serial.print("Time already elapsed (min): ");
    Serial.println((nowSec - eveningOnSec) / 60);
    Serial.print("Remaining (min): ");
    Serial.println((eveningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Evening: boot AFTER window closed AND never completed today → catch up ──
  else if (nowSec >= eveningOffSec && getEveningCompletedDay() != today) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now();   // full fresh duration starts now
    Serial.println(">>> CATCH-UP: Evening window was missed entirely — running full duration now");
    didSomething = true;
  }

  if (!didSomething) {
    Serial.println("Boot outside any ON window and today's cycles already completed — relay stays OFF");
  }

  // NOTE: relayMorningActive and relayEveningActive can technically both be
  // true at once during a catch-up (e.g. reboot very late at night after
  // BOTH windows were missed). Since there is only one physical relay pin,
  // if that happens the relay stays ON until BOTH durations have elapsed —
  // the loop()'s OFF logic below only writes RELAY HIGH once neither flag
  // is active-and-unexpired, so this is handled safely, just FYI.
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);   // relay OFF initially

  Serial.begin(9600);
  while (!Serial);

  if (!RTC.begin()) {
    Serial.println("RTC not found! Check wiring.");
    while (1);
  }
  Serial.println("RTC OK");

  setSyncProvider(getTime);
  setSyncInterval(300);

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
  checkRelayRecovery();   // then recovery / catch-up uses those values
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  Alarm.delay(1000);
  digitalClockDisplay();

  DateTime dt = RTC.now();
  if (dt.day() != lastCalcDay) {
    Serial.println("New day — recalculating solar times...");
    calcTodayAlarms();
  }

  // Morning OFF
  if (relayMorningActive && now() - relayOnTimeMorning >= MORNING_DURATION_MIN * 60L) {
    relayMorningActive = false;
    markMorningCompleted(dt.day());
    if (!relayEveningActive) digitalWrite(RELAY, HIGH);   // don't cut evening off early
    Serial.println("Morning OFF");
  }

  // Evening OFF
  if (relayEveningActive && now() - relayOnTimeEvening >= EVENING_DURATION_MIN * 60L) {
    relayEveningActive = false;
    markEveningCompleted(dt.day());
    if (!relayMorningActive) digitalWrite(RELAY, HIGH);   // don't cut morning off early
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

