#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <TimeAlarms.h>

const int Relay = 7;
bool relayMorningActive = false;
bool relayEveningActive = false;
time_t relayOnTimeMorning = 0;
time_t relayOnTimeEvening = 0;

RTC_DS3231 RTC;

// Bridge function: RTClib → TimeLib
time_t getTime() {
  DateTime now = RTC.now();
  return now.unixtime();
}

void setup() {
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, HIGH);
  Serial.begin(9600);
  while (!Serial);

  if (!RTC.begin()) {
    Serial.println("Unable to sync with the RTC");
    while (1);
  } else {
    Serial.println("RTC has set the system time");
  }

  setSyncProvider(getTime);   // feed RTClib time into TimeLib
  setSyncInterval(300);       // re-sync every 5 minutes

  Alarm.alarmRepeat(5, 30, 0, MorningAlarm);   // 5:30 AM
  Alarm.alarmRepeat(18, 0, 0, EveningAlarm);   // 6:00 PM

  checkRelayRecovery();
}

void loop() {
  Alarm.delay(1000);
  digitalClockDisplay();

  if (relayMorningActive && now() - relayOnTimeMorning >= 1800) {
    digitalWrite(Relay, HIGH);
    relayMorningActive = false;
    Serial.println("Morning OFF (after 30 minutes)");
  }

  if (relayEveningActive && now() - relayOnTimeEvening >= 14400) {
    digitalWrite(Relay, HIGH);
    relayEveningActive = false;
    Serial.println("Evening OFF (after 4 hours)");
  }
}

void MorningAlarm() {
  Serial.println("Morning ON");
  digitalWrite(Relay, LOW);
  relayMorningActive = true;
  relayOnTimeMorning = now();
}

void EveningAlarm() {
  Serial.println("Evening ON");
  digitalWrite(Relay, LOW);
  relayEveningActive = true;
  relayOnTimeEvening = now();
}

void digitalClockDisplay() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println();
}

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10) Serial.print('0');
  Serial.print(digits);
}

void checkRelayRecovery() {
  time_t tNow = now();
  time_t todayMorningOn = previousMidnight(tNow) + 5L * 3600 + 30L * 60;
  time_t todayEveningOn = previousMidnight(tNow) + 18L * 3600;

  if (tNow >= todayMorningOn && tNow < todayMorningOn + 1800) {
    digitalWrite(Relay, LOW);
    relayMorningActive = true;
    relayOnTimeMorning = todayMorningOn;
    Serial.println("Recovered Morning relay ON after reboot");
  }
  if (tNow >= todayEveningOn && tNow < todayEveningOn + 14400) {
    digitalWrite(Relay, LOW);
    relayEveningActive = true;
    relayOnTimeEvening = todayEveningOn;
    Serial.println("Recovered Evening relay ON after reboot");
  }
}
