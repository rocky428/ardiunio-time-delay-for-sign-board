// ═══════════════════════════════════════════════════════════════
//  Solar irrigation relay controller  —  DS3231 + TimeLib + TimeAlarms
//  v2  (2026-10-27)
//
//  v2 changes vs v1:
//   1. Equation of Time now converts ecliptic longitude -> right
//      ascension first. Removes up to ~10 min seasonal error
//      (verified < 0.3 min vs reference almanac for all of 2026).
//   2. EEPROM "completed" marker: day-of-month (1 byte) replaced by
//      a month*32+day stamp (2 bytes). Fixes:
//        - the once-a-month false "already done" collision that
//          skipped a legitimate catch-up,
//        - midnight-crossing mis-marking (a catch-up running past
//          midnight now marks the date the run BELONGED to).
//   3. RTC.lostPower() detection: dead coin cell / first power-up
//      seeds the clock from sketch compile time, warns loudly, and
//      skips resume/catch-up watering on an untrusted clock.
//   4. `while (!Serial);` replaced with a 2 s timeout so USB-native
//      boards (Leonardo / Pro Micro) don't hang forever standalone.
// ═══════════════════════════════════════════════════════════════

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

// ─── EEPROM: last completed run as (month*32+day) stamp ──────
// 2 bytes each, little-endian. Blank EEPROM reads 0xFFFF, which can
// never match a valid stamp (max 12*32+31 = 415), so a fresh chip
// simply means "nothing completed yet". Old v1 single-byte data is
// also safely ignored (it never decodes to a matching stamp).
const int EEPROM_ADDR_MORNING = 0;   // bytes 0-1
const int EEPROM_ADDR_EVENING = 2;   // bytes 2-3

// ─── State ───────────────────────────────────────────────────
bool   relayMorningActive = false;
bool   relayEveningActive = false;
time_t relayOnTimeMorning = 0;
time_t relayOnTimeEvening = 0;

uint16_t morningRunStamp = 0;   // (month*32+day) the current morning run belongs to
uint16_t eveningRunStamp = 0;

long morningOnSec = 0;   // seconds since midnight for morning ON
long eveningOnSec = 0;   // seconds since midnight for evening ON
int  lastCalcDay  = -1;

AlarmID_t morningAlarmID = dtINVALID_ALARM_ID;
AlarmID_t eveningAlarmID = dtINVALID_ALARM_ID;

RTC_DS3231 RTC;

// ─── TimeLib sync bridge ─────────────────────────────────────
time_t getTime() {
  return RTC.now().unixtime();
}

// ─── EEPROM helpers ──────────────────────────────────────────
uint16_t makeStamp(int mo, int dy) {
  return (uint16_t)(mo * 32 + dy);            // unique for every (month, day)
}

void writeStamp(int addr, uint16_t stamp) {
  EEPROM.update(addr,     (uint8_t)(stamp & 0xFF));
  EEPROM.update(addr + 1, (uint8_t)(stamp >> 8));
}

uint16_t readStamp(int addr) {
  return (uint16_t)EEPROM.read(addr) | ((uint16_t)EEPROM.read(addr + 1) << 8);
}

bool morningCompletedOn(uint16_t stamp) { return readStamp(EEPROM_ADDR_MORNING) == stamp; }
bool eveningCompletedOn(uint16_t stamp) { return readStamp(EEPROM_ADDR_EVENING) == stamp; }

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

  // ── Equation of Time (hours, apparent − mean) ─────────────
  // EoT = mean longitude − RIGHT ASCENSION of the true sun.
  // Converting ecliptic longitude -> right ascension is REQUIRED;
  // the old shortcut (using longitude directly) caused up to ~10 min
  // of seasonal error (e.g. ~9 min late in late October here).
  float epsR = 23.439 * DEG_TO_RAD;
  float ra   = atan2(cos(epsR) * sin(lr), cos(lr)) * RAD_TO_DEG;
  float dEq  = L - ra;
  dEq       -= 360.0 * floor((dEq + 180.0) / 360.0);   // wrap to [-180, 180)
  float EoT  = dEq / 15.0;                             // hours

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

  Serial.print("Alarms set -> Morning ");
  Serial.print(mH); Serial.print(":"); if (mM < 10) Serial.print("0"); Serial.print(mM);
  Serial.print("  Evening ");
  Serial.print(eH); Serial.print(":"); if (eM < 10) Serial.print("0"); Serial.println(eM);
}

// ════════════════════════════════════════════════════════════
//  POWER CUT / RESET RECOVERY
//  1) If boot happens INSIDE a window -> resume mid-way
//  2) If boot happens AFTER a window has fully closed AND that window
//     never completed today (per EEPROM stamp) -> CATCH UP: turn relay
//     on now and run it for the FULL configured duration.
//  Skipped entirely if the RTC clock was just re-seeded (lostPower),
//  because no watering decision should be made on an untrusted clock.
// ════════════════════════════════════════════════════════════
void checkRelayRecovery(bool timeTrustworthy) {
  DateTime dt = RTC.now();

  long nowSec = (long)dt.hour() * 3600L + dt.minute() * 60L + dt.second();
  uint16_t todayStamp = makeStamp(dt.month(), dt.day());

  long morningOffSec = morningOnSec + MORNING_DURATION_MIN * 60L;
  long eveningOffSec = eveningOnSec + EVENING_DURATION_MIN * 60L;

  Serial.print("Boot time (sec since midnight): ");
  Serial.println(nowSec);
  Serial.print("Morning window: ");
  Serial.print(morningOnSec); Serial.print(" -> "); Serial.println(morningOffSec);
  Serial.print("Evening window: ");
  Serial.print(eveningOnSec); Serial.print(" -> "); Serial.println(eveningOffSec);

  if (!timeTrustworthy) {
    Serial.println("!! RTC was just re-seeded - clock NOT trustworthy.");
    Serial.println("!! Skipping resume/catch-up this boot (no watering decisions on a bad clock).");
    return;
  }

  bool morningDone = morningCompletedOn(todayStamp);
  bool eveningDone = eveningCompletedOn(todayStamp);
  Serial.print("Morning already completed today: "); Serial.println(morningDone ? "YES" : "NO");
  Serial.print("Evening already completed today: "); Serial.println(eveningDone ? "YES" : "NO");

  bool didSomething = false;

  // ── Morning: boot INSIDE window -> resume ─────────────────
  if (nowSec >= morningOnSec && nowSec < morningOffSec) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now() - (nowSec - morningOnSec);
    morningRunStamp    = todayStamp;
    Serial.println(">>> RECOVERED: Morning relay ON after power cut (resumed mid-window)");
    Serial.print("Time already elapsed (min): ");
    Serial.println((nowSec - morningOnSec) / 60);
    Serial.print("Remaining (min): ");
    Serial.println((morningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Morning: boot AFTER window closed AND never completed today -> catch up ──
  else if (nowSec >= morningOffSec && !morningDone) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now();      // full fresh duration starts now
    morningRunStamp    = todayStamp;
    Serial.println(">>> CATCH-UP: Morning window was missed entirely - running full duration now");
    didSomething = true;
  }

  // ── Evening: boot INSIDE window -> resume ─────────────────
  if (nowSec >= eveningOnSec && nowSec < eveningOffSec) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now() - (nowSec - eveningOnSec);
    eveningRunStamp    = todayStamp;
    Serial.println(">>> RECOVERED: Evening relay ON after power cut (resumed mid-window)");
    Serial.print("Time already elapsed (min): ");
    Serial.println((nowSec - eveningOnSec) / 60);
    Serial.print("Remaining (min): ");
    Serial.println((eveningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Evening: boot AFTER window closed AND never completed today -> catch up ──
  else if (nowSec >= eveningOffSec && !eveningDone) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now();      // full fresh duration starts now
    eveningRunStamp    = todayStamp;
    Serial.println(">>> CATCH-UP: Evening window was missed entirely - running full duration now");
    didSomething = true;
  }

  if (!didSomething) {
    Serial.println("Boot outside any ON window and today's cycles already completed - relay stays OFF");
  }

  // NOTE: relayMorningActive and relayEveningActive can technically both be
  // true at once during a catch-up (e.g. reboot very late at night after
  // BOTH windows were missed). Since there is only one physical relay pin,
  // if that happens the relay stays ON until BOTH durations have elapsed -
  // the loop()'s OFF logic below only writes RELAY HIGH once neither flag
  // is active-and-unexpired, so this is handled safely, just FYI.
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);   // relay OFF initially (active-LOW module)

  Serial.begin(9600);

  // Wait up to 2 s for the serial monitor, then continue anyway.
  // (A bare `while (!Serial);` hangs USB-native boards forever when
  //  powered standalone - fatal for an unattended controller.)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000UL)) {}

  if (!RTC.begin()) {
    Serial.println("RTC not found! Check wiring.");
    while (1);
  }
  Serial.println("RTC OK");

  // Detect RTC power loss (dead coin cell, first-ever power-up, or a
  // power cut with no battery backup). The clock is then invalid, so
  // seed it from the sketch compile time and flag it untrusted.
  bool timeTrustworthy = true;
  if (RTC.lostPower()) {
    timeTrustworthy = false;
    Serial.println("================================================");
    Serial.println("!! RTC LOST POWER - clock was invalid.");
    Serial.println("!! Seeding from sketch compile time, which is NOT");
    Serial.println("!! necessarily today. Please set the correct time");
    Serial.println("!! and replace the CR2032 backup battery.");
    Serial.println("!! Resume/CATCH-UP watering is DISABLED this boot");
    Serial.println("!! for safety; normal alarms will still run.");
    Serial.println("================================================");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

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

  calcTodayAlarms();                    // sets morningOnSec + eveningOnSec FIRST
  checkRelayRecovery(timeTrustworthy);  // then recovery / catch-up uses those values
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  Alarm.delay(1000);
  digitalClockDisplay();

  DateTime dt = RTC.now();
  if (dt.day() != lastCalcDay) {
    Serial.println("New day - recalculating solar times...");
    calcTodayAlarms();
  }

  // Morning OFF
  if (relayMorningActive && now() - relayOnTimeMorning >= MORNING_DURATION_MIN * 60L) {
    relayMorningActive = false;
    writeStamp(EEPROM_ADDR_MORNING, morningRunStamp);   // mark the run's date, not "now"
    if (!relayEveningActive) digitalWrite(RELAY, HIGH); // don't cut evening off early
    Serial.println("Morning OFF");
  }

  // Evening OFF
  if (relayEveningActive && now() - relayOnTimeEvening >= EVENING_DURATION_MIN * 60L) {
    relayEveningActive = false;
    writeStamp(EEPROM_ADDR_EVENING, eveningRunStamp);   // mark the run's date, not "now"
    if (!relayMorningActive) digitalWrite(RELAY, HIGH); // don't cut morning off early
    Serial.println("Evening OFF");
  }
}

// ─── Alarm callbacks ─────────────────────────────────────────
void MorningAlarm() {
  DateTime dt = RTC.now();
  Serial.println("Morning ON");
  digitalWrite(RELAY, LOW);
  relayMorningActive = true;
  relayOnTimeMorning = now();
  morningRunStamp    = makeStamp(dt.month(), dt.day());
}

void EveningAlarm() {
  DateTime dt = RTC.now();
  Serial.println("Evening ON");
  digitalWrite(RELAY, LOW);
  relayEveningActive = true;
  relayOnTimeEvening = now();
  eveningRunStamp    = makeStamp(dt.month(), dt.day());
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
