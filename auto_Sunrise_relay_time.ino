// ═══════════════════════════════════════════════════════════════
//  Solar irrigation relay controller — DS3231 + TimeLib + TimeAlarms
//  v4  (2026-11-03)
//
//  Based on v3. Changes in v4:
//
//   1. FAIL-SAFE CLOCK TRUST
//      A "clock confirmed" flag is stored in EEPROM. On the first
//      boot after flashing, after a battery loss, or when TimeLib
//      fails to sync, alarms are NOT scheduled and catch-up is
//      disabled until the clock is set with the serial command.
//      This prevents wrong-time watering and also prevents the
//      spurious first-boot catch-up caused by upgrading from v1/v3
//      (their EEPROM format is different).
//
//   2. EEPROM STAMP NOW INCLUDES THE YEAR
//      stamp = (year - 2000) * 512 + month * 32 + day
//      Valid range 33..51103, while blank EEPROM reads 0xFFFF.
//      Old v1/v3 values can never equal a 2026+ stamp, so upgrading
//      cannot produce a false "already completed today".
//
//   3. RELAY BOOT GLITCH FIXED
//      digitalWrite(HIGH) happens before pinMode(OUTPUT), so an
//      active-LOW relay module does not click ON at power-up.
//
//   4. SAFE MANUAL TIME SET
//      "T..." re-plans the alarms but does NOT trigger a catch-up.
//      If a run is active, its start time is shifted by the clock
//      delta, so its remaining duration is preserved. If the new
//      time is inside a window, that window is resumed.
//
//   5. RTC / TIME VALIDATION
//      Full date validation (days per month + leap year), full
//      date/time printing, "S" status and "?" help commands.
//
//   6. FULL-DATE DAY ROLLOVER
//      Uses year+month+day instead of day-of-month only.
//
//   7. UNTRUSTED CLOCK = NO ALARMS
//      The relay stays OFF and no alarms are scheduled until the
//      clock is confirmed. Better to skip watering than to water
//      at the wrong time.
//
//  Serial commands (9600 baud, newline terminated):
//      T2026-11-03 17:30:00     set clock
//      T2026/11/03 17:30        '/' separator and no seconds also OK
//      S                        print status
//      ?                        help
//
//  IMPORTANT: the DS3231 must be set to LOCAL time (IST), not UTC.
//  Replace the CR2032/LIR2032 backup cell if lostPower is reported.
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <EEPROM.h>
#include <stdio.h>

// ─── Your location (Udumalpet area, Tamil Nadu) ──────────────
const float LAT =  10.57997;
const float LON =  77.24842;
const float TZ  =  5.5;        // IST = UTC+5:30

// ─── Relay ───────────────────────────────────────────────────
const int RELAY = 7;           // active-LOW module: LOW = ON

// ─── Config ──────────────────────────────────────────────────
const int MORNING_BEFORE_SUNRISE_MIN = 60;
const int MORNING_DURATION_MIN       = 60;
const int EVENING_DURATION_MIN       = 240;

// ─── EEPROM layout ───────────────────────────────────────────
//  0-1 : morning completion stamp (uint16, little-endian)
//  2-3 : evening completion stamp
//  4   : clock-confirmed flag (0xC5 = confirmed)
const int     EEPROM_ADDR_MORNING  = 0;
const int     EEPROM_ADDR_EVENING  = 2;
const int     EEPROM_ADDR_CLOCK_OK = 4;
const uint8_t CLOCK_OK_MAGIC       = 0xC5;

// ─── State ───────────────────────────────────────────────────
bool clockTrusted = false;

bool     relayMorningActive = false;
bool     relayEveningActive = false;
time_t   relayOnTimeMorning = 0;
time_t   relayOnTimeEvening = 0;
uint16_t morningRunStamp    = 0;
uint16_t eveningRunStamp    = 0;

long     morningOnSec  = -1;   // seconds since midnight
long     eveningOnSec  = -1;
float    lastSunriseMin = -1;
float    lastDuskMin    = -1;
uint32_t lastCalcDate   = 0;

AlarmID_t morningAlarmID = dtINVALID_ALARM_ID;
AlarmID_t eveningAlarmID = dtINVALID_ALARM_ID;

RTC_DS3231 RTC;

// ─── Prototypes ──────────────────────────────────────────────
void MorningAlarm();
void EveningAlarm();
void calcTodayAlarms();
void checkRelayRecovery(bool timeTrustworthy, bool allowCatchUp);
void processCommand(char *s);
void processSetTimeCmd(char *s);
void printStatus();
void printHelp();
void handleSerialCommands();
void digitalClockDisplay();
float calcSunEvent(int yr, int mo, int dy, float lat, float lon, float tz, int type);

void freeAlarms();
void print2(int v);
void printHm(int totalMin);
void printHms(long s);
void printDateTime(const DateTime &dt);
bool validDateTime(int Y, int M, int D, int h, int m, int s);
bool elapsedReached(time_t start, long seconds);

uint16_t makeStamp(int yr, int mo, int dy);
void     writeStamp(int addr, uint16_t stamp);
uint16_t readStamp(int addr);
bool     morningCompletedOn(uint16_t stamp);
bool     eveningCompletedOn(uint16_t stamp);
bool     clockValidFlag();
void     setClockValidFlag(bool ok);

uint32_t dateKey(const DateTime &dt);

// ─── TimeLib sync bridge ─────────────────────────────────────
time_t getTime() {
  return RTC.now().unixtime();
}

// ─── EEPROM helpers ──────────────────────────────────────────
uint16_t makeStamp(int yr, int mo, int dy) {
  // 32-bit arithmetic on purpose: AVR 'int' is only 16 bits and
  // (yr - 2000) * 512 would overflow for years past ~2064.
  return (uint16_t)(((uint32_t)(yr - 2000) * 512UL) + (uint32_t)(mo * 32 + dy));
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

bool clockValidFlag() {
  return EEPROM.read(EEPROM_ADDR_CLOCK_OK) == CLOCK_OK_MAGIC;
}

void setClockValidFlag(bool ok) {
  EEPROM.update(EEPROM_ADDR_CLOCK_OK, ok ? CLOCK_OK_MAGIC : 0x00);
}

uint32_t dateKey(const DateTime &dt) {
  return (uint32_t)dt.year() * 10000UL + (uint32_t)dt.month() * 100UL + dt.day();
}

// ─── Small print helpers ─────────────────────────────────────
void print2(int v) {
  if (v < 10) Serial.print('0');
  Serial.print(v);
}

void printHm(int totalMin) {
  if (totalMin < 0) { Serial.print(F("n/a")); return; }
  print2(totalMin / 60);
  Serial.print(':');
  print2(totalMin % 60);
}

void printHms(long s) {
  if (s < 0) { Serial.print(F("n/a")); return; }
  print2((int)(s / 3600));
  Serial.print(':');
  print2((int)((s % 3600) / 60));
  Serial.print(':');
  print2((int)(s % 60));
}

void printDateTime(const DateTime &dt) {
  Serial.print(dt.year());
  Serial.print('-');
  print2(dt.month());
  Serial.print('-');
  print2(dt.day());
  Serial.print(' ');
  print2(dt.hour());
  Serial.print(':');
  print2(dt.minute());
  Serial.print(':');
  print2(dt.second());
}

bool elapsedReached(time_t start, long seconds) {
  int32_t diff = (int32_t)(now() - start);
  return diff >= seconds;
}

// ═══════════════════════════════════════════════════════════════
//  SOLAR CALCULATOR  (Jean Meeus simplified)
//  type: 0 = sunrise, 1 = civil dusk (sun 6° below horizon)
//  Returns local minutes since midnight, or -1 if no event.
//  v3/v4 equation of time: longitude -> right ascension.
// ═══════════════════════════════════════════════════════════════
float calcSunEvent(int yr, int mo, int dy, float lat, float lon, float tz, int type) {
  int a  = (14 - mo) / 12;
  int y  = yr + 4800 - a;
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

  // Equation of Time (hours, apparent - mean)
  float epsR = 23.439 * DEG_TO_RAD;
  float ra   = atan2(cos(epsR) * sin(lr), cos(lr)) * RAD_TO_DEG;
  float dEq  = L - ra;
  dEq       -= 360.0 * floor((dEq + 180.0) / 360.0);   // wrap to [-180, 180)
  float EoT  = dEq / 15.0;

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

// ═══════════════════════════════════════════════════════════════
//  SERIAL COMMANDS
// ═══════════════════════════════════════════════════════════════
char    cmdBuf[32];
uint8_t cmdLen      = 0;
bool    cmdOverflow = false;

void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (cmdOverflow) {
        Serial.println(F("!! Command too long, discarded."));
      } else if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        processCommand(cmdBuf);
      }
      cmdLen      = 0;
      cmdOverflow = false;
    } else if (cmdOverflow) {
      // discard until end of line
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = true;
    }
  }
}

void processCommand(char *s) {
  switch (s[0]) {
    case 'T': case 't': processSetTimeCmd(s); break;
    case 'S': case 's': printStatus();        break;
    case '?': case 'H': case 'h': printHelp(); break;
    default:
      Serial.print(F("?? Unknown command: "));
      Serial.println(s);
      printHelp();
      break;
  }
}

bool validDateTime(int Y, int M, int D, int h, int m, int s) {
  if (Y < 2000 || Y > 2099 || M < 1 || M > 12 ||
      h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
    return false;
  }

  static const uint8_t dim[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  uint8_t maxD = dim[M - 1];

  if (M == 2 && ((Y % 4 == 0 && Y % 100 != 0) || (Y % 400 == 0))) {
    maxD = 29;
  }

  return D >= 1 && D <= maxD;
}

void processSetTimeCmd(char *s) {
  int Y = 0, M = 0, D = 0, h = 0, m = 0, sec = 0;

  int n = sscanf(s + 1, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec);
  if (n < 5) {
    Y = M = D = h = m = sec = 0;
    n = sscanf(s + 1, "%d/%d/%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec);
  }

  if (n == 5) {
    sec = 0;                       // seconds omitted
  } else if (n != 6) {
    Serial.println(F("!! Could not parse. Example: T2026-11-03 17:30:00"));
    return;
  }

  if (!validDateTime(Y, M, D, h, m, sec)) {
    Serial.println(F("!! Date/time out of range (or invalid day for that month)."));
    return;
  }

  time_t before = now();

  RTC.adjust(DateTime((uint16_t)Y, (uint8_t)M, (uint8_t)D,
                      (uint8_t)h, (uint8_t)m, (uint8_t)sec));
  setTime(h, m, sec, D, M, Y);     // update TimeLib immediately

  time_t after  = now();
  int32_t delta = (int32_t)(after - before);

  // Keep any active run's remaining duration unchanged across the jump.
  if (relayMorningActive) relayOnTimeMorning += delta;
  if (relayEveningActive) relayOnTimeEvening += delta;

  clockTrusted = true;
  setClockValidFlag(true);

  Serial.print(F("OK - clock set to "));
  DateTime dt = RTC.now();
  printDateTime(dt);
  Serial.println();

  calcTodayAlarms();               // re-plan alarms for the new date

  if (relayMorningActive || relayEveningActive) {
    Serial.println(F("(Run in progress - start time shifted, remaining duration kept)"));
  } else {
    // Resume only if the new time is inside a window.
    // Deliberately do NOT catch up a missed window here.
    checkRelayRecovery(true, false);
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  T2026-11-03 17:30:00   set clock (local IST)"));
  Serial.println(F("  T2026/11/03 17:30      '/' and no seconds also OK"));
  Serial.println(F("  S                      status"));
  Serial.println(F("  ?                      this help"));
}

void printStatus() {
  DateTime dt = RTC.now();

  Serial.print(F("Now          : "));
  printDateTime(dt);
  Serial.println();

  Serial.print(F("lostPower    : "));
  Serial.println(RTC.lostPower() ? F("YES") : F("no"));

  Serial.print(F("clockTrusted : "));
  Serial.println(clockTrusted ? F("YES") : F("NO"));

  Serial.print(F("Sunrise      : "));
  if (lastSunriseMin < 0) { Serial.println(F("n/a")); }
  else { printHm((int)lastSunriseMin); Serial.println(); }

  Serial.print(F("Civil dusk   : "));
  if (lastDuskMin < 0) { Serial.println(F("n/a")); }
  else { printHm((int)lastDuskMin); Serial.println(); }

  Serial.print(F("Morning win  : "));
  printHms(morningOnSec);
  Serial.print(F(" -> "));
  printHms(morningOnSec >= 0 ? morningOnSec + MORNING_DURATION_MIN * 60L : -1);
  Serial.println();

  Serial.print(F("Evening win  : "));
  printHms(eveningOnSec);
  Serial.print(F(" -> "));
  printHms(eveningOnSec >= 0 ? eveningOnSec + EVENING_DURATION_MIN * 60L : -1);
  Serial.println();

  uint16_t stamp = makeStamp(dt.year(), dt.month(), dt.day());
  Serial.print(F("Today stamp  : "));
  Serial.println(stamp);

  Serial.print(F("Morning done : "));
  Serial.println(morningCompletedOn(stamp) ? F("YES") : F("NO"));

  Serial.print(F("Evening done : "));
  Serial.println(eveningCompletedOn(stamp) ? F("YES") : F("NO"));

  Serial.print(F("Relay flags  : morning="));
  Serial.print(relayMorningActive ? F("ON") : F("off"));
  Serial.print(F(" evening="));
  Serial.print(relayEveningActive ? F("ON") : F("off"));
  Serial.print(F("  pin="));
  Serial.println(digitalRead(RELAY) == LOW ? F("ON") : F("OFF"));
}

// ═══════════════════════════════════════════════════════════════
//  ALARM PLANNING
// ═══════════════════════════════════════════════════════════════
void freeAlarms() {
  if (morningAlarmID != dtINVALID_ALARM_ID) {
    Alarm.free(morningAlarmID);
    morningAlarmID = dtINVALID_ALARM_ID;
  }
  if (eveningAlarmID != dtINVALID_ALARM_ID) {
    Alarm.free(eveningAlarmID);
    eveningAlarmID = dtINVALID_ALARM_ID;
  }
}

void calcTodayAlarms() {
  DateTime dt = RTC.now();
  int yr = dt.year();
  int mo = dt.month();
  int dy = dt.day();

  lastCalcDate = dateKey(dt);

  float sunriseMins = calcSunEvent(yr, mo, dy, LAT, LON, TZ, 0);
  float civilDuskMin = calcSunEvent(yr, mo, dy, LAT, LON, TZ, 1);

  lastSunriseMin = sunriseMins;
  lastDuskMin    = civilDuskMin;

  freeAlarms();

  if (sunriseMins < 0 || civilDuskMin < 0) {
    morningOnSec = -1;
    eveningOnSec = -1;
    Serial.println(F("!! No sunrise/dusk for this date - alarms disabled."));
    return;
  }

  float morningMins = sunriseMins - MORNING_BEFORE_SUNRISE_MIN;
  if (morningMins < 0) morningMins += 1440;

  morningOnSec = (long)(morningMins  * 60);
  eveningOnSec = (long)(civilDuskMin * 60);

  Serial.print(F("Sunrise      : ")); printHm((int)sunriseMins); Serial.println();
  Serial.print(F("Civil dusk   : ")); printHm((int)civilDuskMin); Serial.println();
  Serial.print(F("Morning ON   : ")); printHm((int)morningMins); Serial.println();
  Serial.print(F("Evening ON   : ")); printHm((int)civilDuskMin); Serial.println();

  if (!clockTrusted) {
    Serial.println(F("Clock untrusted - alarms NOT scheduled."));
    Serial.println(F("Send: T2026-11-03 17:30:00"));
    return;
  }

  int mH = (int)(morningOnSec / 3600);
  int mM = (int)((morningOnSec % 3600) / 60);
  int eH = (int)(eveningOnSec / 3600);
  int eM = (int)((eveningOnSec % 3600) / 60);

  morningAlarmID = Alarm.alarmRepeat(mH, mM, 0, MorningAlarm);
  eveningAlarmID = Alarm.alarmRepeat(eH, eM, 0, EveningAlarm);

  Serial.print(F("Alarms set   : Morning "));
  print2(mH); Serial.print(':');
  print2(mM);
  Serial.print(F("  Evening "));
  print2(eH); Serial.print(':');
  print2(eM);
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════
//  BOOT / POWER-CUT RECOVERY
//  1) Boot INSIDE a window            -> resume mid-way
//  2) Boot AFTER a window, not done   -> catch up full duration
//     (only when allowCatchUp is true)
//  Alarms/catch-up are skipped entirely if the clock is untrusted.
// ═══════════════════════════════════════════════════════════════
void checkRelayRecovery(bool timeTrustworthy, bool allowCatchUp) {
  if (morningOnSec < 0 || eveningOnSec < 0) return;

  DateTime dt = RTC.now();
  if (dt.year() < 2000) {
    Serial.println(F("!! RTC year is invalid - recovery skipped."));
    return;
  }

  long nowSec = (long)dt.hour() * 3600L + dt.minute() * 60L + dt.second();
  uint16_t todayStamp = makeStamp(dt.year(), dt.month(), dt.day());

  long morningOffSec = morningOnSec + MORNING_DURATION_MIN * 60L;
  long eveningOffSec = eveningOnSec + EVENING_DURATION_MIN * 60L;

  Serial.print(F("Boot time    : ")); printHms(nowSec); Serial.println();
  Serial.print(F("Morning win  : ")); printHms(morningOnSec);
  Serial.print(F(" -> ")); printHms(morningOffSec); Serial.println();
  Serial.print(F("Evening win  : ")); printHms(eveningOnSec);
  Serial.print(F(" -> ")); printHms(eveningOffSec); Serial.println();

  if (!timeTrustworthy) {
    Serial.println(F("!! Clock is NOT trustworthy - no resume/catch-up."));
    Serial.println(F("!! Set the clock: T2026-11-03 17:30:00"));
    return;
  }

  bool morningDone = morningCompletedOn(todayStamp);
  bool eveningDone = eveningCompletedOn(todayStamp);

  Serial.print(F("Morning done : ")); Serial.println(morningDone ? F("YES") : F("NO"));
  Serial.print(F("Evening done : ")); Serial.println(eveningDone ? F("YES") : F("NO"));

  bool didSomething = false;

  // ── Morning: inside window -> resume ──────────────────────
  if (!morningDone && nowSec >= morningOnSec && nowSec < morningOffSec) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now() - (nowSec - morningOnSec);
    morningRunStamp    = todayStamp;
    Serial.println(F(">>> RESUME: Morning relay ON (remaining part of window)"));
    Serial.print(F("    Remaining (min): "));
    Serial.println((morningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Morning: after window, never done -> catch up ─────────
  else if (!morningDone && allowCatchUp && nowSec >= morningOffSec) {
    digitalWrite(RELAY, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = now();
    morningRunStamp    = todayStamp;
    Serial.println(F(">>> CATCH-UP: Morning missed - running full duration now"));
    didSomething = true;
  }

  // ── Evening: inside window -> resume ──────────────────────
  if (!eveningDone && nowSec >= eveningOnSec && nowSec < eveningOffSec) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now() - (nowSec - eveningOnSec);
    eveningRunStamp    = todayStamp;
    Serial.println(F(">>> RESUME: Evening relay ON (remaining part of window)"));
    Serial.print(F("    Remaining (min): "));
    Serial.println((eveningOffSec - nowSec) / 60);
    didSomething = true;
  }
  // ── Evening: after window, never done -> catch up ─────────
  else if (!eveningDone && allowCatchUp && nowSec >= eveningOffSec) {
    digitalWrite(RELAY, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = now();
    eveningRunStamp    = todayStamp;
    Serial.println(F(">>> CATCH-UP: Evening missed - running full duration now"));
    didSomething = true;
  }

  if (!didSomething) {
    if (nowSec < morningOnSec) {
      Serial.println(F("Boot is before today's windows - waiting for alarms."));
    } else if (!allowCatchUp) {
      Serial.println(F("Boot is after a window; catch-up suppressed for this call."));
    } else {
      Serial.println(F("No action needed - today's cycles are already completed."));
    }
  }
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  // Drive HIGH first, then make it an output: avoids the brief LOW
  // glitch an active-LOW relay board would otherwise see at boot.
  digitalWrite(RELAY, HIGH);
  pinMode(RELAY, OUTPUT);

  Serial.begin(9600);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000UL)) { /* 2 s max */ }

  if (!RTC.begin()) {
    Serial.println(F("RTC not found! Check wiring."));
    Serial.println(F("Relay stays OFF."));
    while (1) { /* fail safe */ }
  }
  Serial.println(F("RTC OK"));

  Serial.print(F("RTC time     : "));
  printDateTime(RTC.now());
  Serial.println();

  // ── Clock trust ──────────────────────────────────────────
  clockTrusted = true;

  if (RTC.lostPower()) {
    clockTrusted = false;
    setClockValidFlag(false);

    Serial.println(F("================================================"));
    Serial.println(F("!! RTC LOST POWER - clock is invalid."));
    Serial.println(F("!! Seeding from compile time for diagnostics only."));
    Serial.println(F("!! Alarms and catch-up are DISABLED until you set"));
    Serial.println(F("!! the clock:  T2026-11-03 17:30:00"));
    Serial.println(F("!! Replace the CR2032/LIR2032 backup cell."));
    Serial.println(F("================================================"));

    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else if (!clockValidFlag()) {
    clockTrusted = false;
    Serial.println(F("!! Clock not confirmed since flashing."));
    Serial.println(F("!! Alarms and catch-up are DISABLED until you set"));
    Serial.println(F("!! the clock:  T2026-11-03 17:30:00"));
  }

  setSyncProvider(getTime);
  setSyncInterval(300);

  int retries = 0;
  while (timeStatus() != timeSet && retries < 10) {
    delay(200);
    retries++;
  }

  if (timeStatus() != timeSet) {
    clockTrusted = false;
    Serial.println(F("!! TimeLib sync failed - alarms disabled."));
  } else {
    Serial.println(F("TimeLib synced from RTC"));
  }

  calcTodayAlarms();                       // sets morningOnSec / eveningOnSec
  checkRelayRecovery(clockTrusted, true);  // boot: catch-up allowed if trusted

  printHelp();
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  Alarm.delay(50);              // service alarms in short slices
  handleSerialCommands();       // keep serial responsive

  static uint32_t lastTick = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastTick < 1000) return;
  lastTick = nowMs;

  digitalClockDisplay();

  DateTime dt = RTC.now();
  if (dateKey(dt) != lastCalcDate) {
    Serial.println(F("New day - recalculating solar times..."));
    calcTodayAlarms();
  }

  // ── Morning OFF ──────────────────────────────────────────
  if (relayMorningActive && elapsedReached(relayOnTimeMorning, MORNING_DURATION_MIN * 60L)) {
    relayMorningActive = false;
    if (morningRunStamp) writeStamp(EEPROM_ADDR_MORNING, morningRunStamp);
    if (!relayEveningActive) digitalWrite(RELAY, HIGH);
    Serial.println(F("Morning OFF"));
  }

  // ── Evening OFF ──────────────────────────────────────────
  if (relayEveningActive && elapsedReached(relayOnTimeEvening, EVENING_DURATION_MIN * 60L)) {
    relayEveningActive = false;
    if (eveningRunStamp) writeStamp(EEPROM_ADDR_EVENING, eveningRunStamp);
    if (!relayMorningActive) digitalWrite(RELAY, HIGH);
    Serial.println(F("Evening OFF"));
  }
}

// ─── Alarm callbacks ─────────────────────────────────────────
void MorningAlarm() {
  if (!clockTrusted) return;
  if (relayMorningActive) return;      // do not restart an active run

  DateTime dt = RTC.now();
  Serial.println(F("Morning ON (alarm)"));
  digitalWrite(RELAY, LOW);
  relayMorningActive = true;
  relayOnTimeMorning = now();
  morningRunStamp    = makeStamp(dt.year(), dt.month(), dt.day());
}

void EveningAlarm() {
  if (!clockTrusted) return;
  if (relayEveningActive) return;

  DateTime dt = RTC.now();
  Serial.println(F("Evening ON (alarm)"));
  digitalWrite(RELAY, LOW);
  relayEveningActive = true;
  relayOnTimeEvening = now();
  eveningRunStamp    = makeStamp(dt.year(), dt.month(), dt.day());
}

// ─── Clock display ───────────────────────────────────────────
void digitalClockDisplay() {
  print2(hour());
  Serial.print(':');
  print2(minute());
  Serial.print(':');
  print2(second());
  Serial.println();
}
