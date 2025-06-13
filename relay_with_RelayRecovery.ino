#include <TimeLib.h>
#include <TimeAlarms.h>
#include <Wire.h>
#include <DS1307RTC.h>

const int Relay = 7;

bool relayMorningActive = false;
bool relayEveningActive = false;
time_t relayOnTimeMorning = 0; // stores time in seconds
time_t relayOnTimeEvening = 0; // stores time in seconds

void setup() {
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, HIGH); // Turn relay OFF initially

  Serial.begin(9600);
  while (!Serial);

  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  // Schedule relay to turn ON
  Alarm.alarmRepeat(5, 30, 0, MorningAlarm);   // 5:30 AM
  Alarm.alarmRepeat(18, 0, 0, EveningAlarm);   // 6:00 PM

  // Check if system restarted during ON window
  checkRelayRecovery();
}

void loop() {
  Alarm.delay(1000); // Wait 1 second between loop cycles
  digitalClockDisplay();

  // Turn OFF relay 30 minutes after Morning ON
  if (relayMorningActive && now() - relayOnTimeMorning >= 1800) { // 1800 seconds = 30 minutes
    digitalWrite(Relay, HIGH); // Turn relay OFF
    relayMorningActive = false;
    Serial.println("Morning OFF (after 30 minutes)");
  }

  // Turn OFF relay 4 hours after Evening ON
  if (relayEveningActive && now() - relayOnTimeEvening >= 14400) { // 14400 seconds = 4 hours
    digitalWrite(Relay, HIGH); // Turn relay OFF
    relayEveningActive = false;
    Serial.println("Evening OFF (after 4 hours)");
  }
}

void MorningAlarm() {
  Serial.println("Morning ON");
  digitalWrite(Relay, LOW); // Turn relay ON
  relayMorningActive = true;
  relayOnTimeMorning = now();
}

void EveningAlarm() {
  Serial.println("Evening ON");
  digitalWrite(Relay, LOW); // Turn relay ON
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

// On boot, check if relay should be ON based on current time
void checkRelayRecovery() {
  time_t tNow = now();
  time_t todayMorningOn = previousMidnight(tNow) + 5L * 3600 + 30L * 60; // 5:30 AM
  time_t todayEveningOn = previousMidnight(tNow) + 18L * 3600;           // 6:00 PM
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

