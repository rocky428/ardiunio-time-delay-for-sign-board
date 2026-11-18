#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

void setup() {
  Serial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // STEP 1: Uncomment the line below to set the date & time, then upload.
  // Format: rtc.adjust(DateTime(YEAR, MONTH, DAY, HOUR, MINUTE, SECOND));
  rtc.adjust(DateTime(2026, 11, 18, 19, 15, 0));

  // STEP 2: After setting the time, add "//" in front of rtc.adjust above 
  // and upload the code a second time so it doesn't reset on reboot.
}

void loop() {
  DateTime now = rtc.now();

  // Print Date (DD/MM/YYYY)
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print("  ");

  // Print Time (HH:MM:SS) with leading zeros
  if (now.hour() < 10) Serial.print('0');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  
  if (now.minute() < 10) Serial.print('0');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  
  if (now.second() < 10) Serial.print('0');
  Serial.print(now.second(), DEC);

  Serial.println();
  delay(1000); // Update every 1 second
}
