// ═══════════════════════════════════════════════════════════════
//  Solar irrigation relay controller  —  DS3231 + TimeLib + TimeAlarms
//  v3  (2026-11-18)
//
//  v3 changes vs v2:
//   1. All Serial text wrapped in F() -> stored in flash, not RAM.
//      Frees roughly 800-1000 bytes (v2 was at 91% RAM on an Uno/Nano).
//   2. Catch-up cutoff times: a missed run is only caught up if the
//      reboot happens before a cutoff (morning: noon, evening: 23:30).
//      Stops a morning cycle starting in the middle of the night.
//   3. Stale-clock check: if the RTC is more than 1 day BEHIND the
//      sketch compile time, the clock is treated as untrusted (this
//      is what happened when the DS3231 silently fell back 2 weeks).
//
//  Unchanged from v2: Equation-of-Time fix, month*32+day EEPROM
//  completion stamps, RTC.lostPower() detection, Serial wait timeout.
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

// Catch-up only if the reboot happens BEFORE these times (sec since midnight)
const long MORNING_CATCHUP_UNTIL_SEC = 12L * 3600L;            // 12:00
const long EVENING_CATCHUP_UNTIL_SEC = 23L * 3600L + 1800L;    // 23:30

// ─── EEPROM: last completed run as (month*32+day) stamp ──────
// 2 bytes each, little-endian. Blank EEPROM reads 0xFFFF, which can
// never match a valid stamp (max 12*32+31 = 415).
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

  // Equation of Time (hours): mean longitude minus RIGHT ASCENSION
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
  Serial.print(':');
  if (m < 10) Serial.print('0');
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

  Serial.print(F("Date         : "));
  Serial.print(dy); Serial.print('/'); Serial.print(mo); Serial.print('/'); Serial.println(yr);
  Serial.print(F("Sunrise      : ")); printMinutes((int)sunriseMins);
  Serial.print(F("Civil dusk   : ")); printMinutes((int)civilDuskMin);
  Serial.print(F("Morning ON   : ")); printMinutes((int)morningMins);
  Serial.print(F("Evening ON   : ")); printMinutes((int)civilDuskMin);

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

  Serial.print(F("Alarms set -> Morning "));
  Serial.print(mH); Serial.print(':'); if (mM < 10) Serial.print('0'); Serial.print(mM);
  Serial.print(F("  Evening "));
  Serial.print(eH); Serial.print(':'); if (eM < 10) Serial.print('0'); Serial.println(eM);
}

// ════════════════════════════════════════════════════════════
//  POWER CUT / RESET RECOVERY
//  1) Boot INSIDE a window            -> resume mid-way
//  2) Boot AFTER a window closed, run never completed today (EEPROM
//     stamp) AND still before the catch-up cutoff -> run FULL duration
//  Skipped entirely if the RTC clock is untrusted.
// ════════════════════════════════════════════════════════════
void checkRelayRecovery(bool timeTrustworthy) {
  DateTime dt = RTC.now();

  long nowSec = (long)dt.hour() * 3600L + dt.minute() * 60L + dt.second();
  uint16_t todayStamp = makeStamp(dt.month(), dt.day());

  long morningOffSec = morningOnSec + MORNING_DURATION_MIN * 60L;
  long eveningOffSec = eveningOnSec + EVENING_DURATION_MIN * 60L;

  Serial.print(F("Boot time (sec since midnight): "));
  Serial.println(nowSec);
  Serial.print(F("Morning window: "));
  Serial.print(morningOnSec); Serial.print(F(" -> ")); Serial.println(morningOffSec);
  Serial.print(F("Evening window: "));
  Serial.print(eveningOnSec); Serial.print(F(" -> ")); Serial.println(eveningOffSec);

  if (!timeTrustworthy) {
    Serial.println(F("!! RTC untrusted - skipping resume/catch-up this boot."));
    return;
  }

  bool morningDone = morningCompletedOn(todayStamp);
  bool eveningDone = eveningCompletedOn(todayStamp);
  Serial.print(F("Morning already completed today: ")); Serial.println(morningDone ? F("YES") : F("NO"));
  Serial.print(F("Evening already completed today: ")); Serial.println(eveningDone ? F("YES") : F("NO"));

  bool didSomething = false;

  // ── Morning: boot INSIDE window -> resume ─────────────────
  if (nowSec >= morningOnSec && nowSec < morningOffSec) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now() - (nowSec - morningOnSec);
    morningRunStamp    = todayStamp;
    Serial.println(F(">>> RECOVERED: Morning relay ON (resumed mid-window)"));
    Serial.print(F("Elapsed (min): "));
    Serial.println((nowSec - morningOnSec) / 60);
    Serial.print(F("Remaining (min): "));
    Serial.println((morningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Morning: missed, not completed, before cutoff -> catch up ──
  else if (nowSec >= morningOffSec && nowSec < MORNING_CATCHUP_UNTIL_SEC && !morningDone) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now();      // full fresh duration starts now
    morningRunStamp    = todayStamp;
    Serial.println(F(">>> CATCH-UP: Morning run missed - running full duration now"));
    didSomething = true;
  }

  // ── Evening: boot INSIDE window -> resume ─────────────────
  if (nowSec >= eveningOnSec && nowSec < eveningOffSec) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now() - (nowSec - eveningOnSec);
    eveningRunStamp    = todayStamp;
    Serial.println(F(">>> RECOVERED: Evening relay ON (resumed mid-window)"));
    Serial.print(F("Elapsed (min): "));
    Serial.println((nowSec - eveningOnSec) / 60);
    Serial.print(F("Remaining (min): "));
    Serial.println((eveningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Evening: missed, not completed, before cutoff -> catch up ──
  else if (nowSec >= eveningOffSec && nowSec < EVENING_CATCHUP_UNTIL_SEC && !eveningDone) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now();      // full fresh duration starts now
    eveningRunStamp    = todayStamp;
    Serial.println(F(">>> CATCH-UP: Evening run missed - running full duration now"));
    didSomething = true;
  }

  if (!didSomething) {
    Serial.println(F("No resume/catch-up needed - relay stays OFF"));
  }

  // NOTE: both flags can be active at once (e.g. catch-up of one run while
  // the other window is open). There is one relay pin, so loop() only
  // writes RELAY HIGH once neither run is still active.
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);   // relay OFF initially (active-LOW module)

  Serial.begin(9600);

  // Wait up to 2 s for the serial monitor, then continue anyway.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000UL)) {}

  if (!RTC.begin()) {
    Serial.println(F("RTC not found! Check wiring."));
    while (1);
  }
  Serial.println(F("RTC OK"));

  // Clock trust checks:
  //  a) RTC.lostPower()  -> oscillator stopped (dead CR2032 / first power-up)
  //  b) RTC more than 1 day behind the sketch compile time -> stale clock
  bool timeTrustworthy = true;
  DateTime compiled(F(__DATE__), F(__TIME__));

  if (RTC.lostPower()) {
    timeTrustworthy = false;
    Serial.println(F("!! RTC LOST POWER - clock invalid."));
  } else if (RTC.now().unixtime() + 86400UL < compiled.unixtime()) {
    timeTrustworthy = false;
    Serial.println(F("!! RTC is far BEHIND compile time - clock stale."));
  }

  if (!timeTrustworthy) {
    Serial.println(F("!! Seeding clock from compile time (may not be today)."));
    Serial.println(F("!! Set the correct time and replace the CR2032."));
    Serial.println(F("!! Resume/catch-up disabled this boot; alarms still run."));
    RTC.adjust(compiled);
  }

  setSyncProvider(getTime);
  setSyncInterval(300);

  int retries = 0;
  while (timeStatus() != timeSet && retries < 10) {
    delay(200);
    retries++;
  }
  if (timeStatus() != timeSet) {
    Serial.println(F("WARNING: TimeLib sync failed, using RTC directly"));
  } else {
    Serial.println(F("TimeLib synced from RTC"));
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
    Serial.println(F("New day - recalculating solar times..."));
    calcTodayAlarms();
  }

  // Morning OFF
  if (relayMorningActive && now() - relayOnTimeMorning >= MORNING_DURATION_MIN * 60L) {
    relayMorningActive = false;
    writeStamp(EEPROM_ADDR_MORNING, morningRunStamp);   // mark the run's date, not "now"
    if (!relayEveningActive) digitalWrite(RELAY, HIGH); // don't cut evening off early
    Serial.println(F("Morning OFF"));
  }

  // Evening OFF
  if (relayEveningActive && now() - relayOnTimeEvening >= EVENING_DURATION_MIN * 60L) {
    relayEveningActive = false;
    writeStamp(EEPROM_ADDR_EVENING, eveningRunStamp);   // mark the run's date, not "now"
    if (!relayMorningActive) digitalWrite(RELAY, HIGH); // don't cut morning off early
    Serial.println(F("Evening OFF"));
  }
}

// ─── Alarm callbacks ─────────────────────────────────────────
void MorningAlarm() {
  DateTime dt = RTC.now();
  Serial.println(F("Morning ON"));
  digitalWrite(RELAY, LOW);
  relayMorningActive = true;
  relayOnTimeMorning = now();
  morningRunStamp    = makeStamp(dt.month(), dt.day());
}

void EveningAlarm() {
  DateTime dt = RTC.now();
  Serial.println(F("Evening ON"));
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
  Serial.print(':');
  if (digits < 10) Serial.print('0');
  Serial.print(digits);
}
