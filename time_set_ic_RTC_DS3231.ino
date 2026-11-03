// ═══════════════════════════════════════════════════════════════
//  set_time_serial.ino — set the DS3231 from the PC's current time
//
//  The Arduino cannot read the PC clock by itself. This sketch
//  listens on serial and accepts the current local date/time from
//  the PC. Use the included set_time.ps1 script to send it
//  automatically, or type it into the Serial Monitor manually:
//
//      T2026-11-03 17:25:00
//      T2026/11/03 17:25        ('/' and no seconds also OK)
//      S                        status
//      ?                        help
//
//  The RTC is set to LOCAL time (e.g. IST for India).
//  Replace the CR2032/LIR2032 backup cell if lostPower is reported,
//  otherwise the time will be lost at the next power cut.
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <RTClib.h>
#include <stdio.h>

RTC_DS3231 RTC;

char    cmdBuf[32];
uint8_t cmdLen      = 0;
bool    cmdOverflow = false;

// ─── Prototypes ──────────────────────────────────────────────
void print2(int v);
void printDateTime(const DateTime &dt);
void printStatus();
void printHelp();
void processCommand(char *s);
void setClockFromCommand(char *s);
bool validDateTime(int Y, int M, int D, int h, int m, int s);

void setup() {
  Serial.begin(9600);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000UL) { }

  if (!RTC.begin()) {
    Serial.println(F("RTC not found! Check wiring."));
    while (1) { }
  }

  Serial.println(F("RTC OK"));
  printStatus();
  printHelp();
}

void loop() {
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
    case 'T': case 't': setClockFromCommand(s); break;
    case 'S': case 's': printStatus();          break;
    case '?': case 'H': case 'h': printHelp();  break;
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

void setClockFromCommand(char *s) {
  int Y = 0, M = 0, D = 0, h = 0, m = 0, sec = 0;

  int n = sscanf(s + 1, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec);
  if (n < 5) {
    Y = M = D = h = m = sec = 0;
    n = sscanf(s + 1, "%d/%d/%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec);
  }

  if (n == 5) {
    sec = 0;                              // seconds omitted
  } else if (n != 6) {
    Serial.println(F("!! Format: T2026-11-03 17:25:00"));
    return;
  }

  if (!validDateTime(Y, M, D, h, m, sec)) {
    Serial.println(F("!! Invalid date/time (check day of month)."));
    return;
  }

  bool lostBefore = RTC.lostPower();

  RTC.adjust(DateTime((uint16_t)Y, (uint8_t)M, (uint8_t)D,
                      (uint8_t)h, (uint8_t)m, (uint8_t)sec));

  DateTime wrote((uint16_t)Y, (uint8_t)M, (uint8_t)D,
                 (uint8_t)h, (uint8_t)m, (uint8_t)sec);
  DateTime readback = RTC.now();

  Serial.print(F("Set to   : ")); printDateTime(wrote);    Serial.println();
  Serial.print(F("Readback : ")); printDateTime(readback); Serial.println();
  Serial.print(F("lostPower before set: "));
  Serial.println(lostBefore ? F("YES - replace the backup cell") : F("no"));
}

void printStatus() {
  DateTime now = RTC.now();
  Serial.print(F("RTC now  : ")); printDateTime(now); Serial.println();
  Serial.print(F("lostPower: "));
  Serial.println(RTC.lostPower() ? F("YES - replace the backup cell") : F("no"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  T2026-11-03 17:25:00   set RTC (local time)"));
  Serial.println(F("  T2026/11/03 17:25      '/' and no seconds also OK"));
  Serial.println(F("  S                      status"));
  Serial.println(F("  ?                      help"));
}

void print2(int v) {
  if (v < 10) Serial.print('0');
  Serial.print(v);
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
