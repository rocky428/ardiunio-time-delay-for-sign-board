/*
  SetTime.ino - Set time for DS3231 RTC
  Adapted from Paul Stoffregen's DS1307RTC example
*/

#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 RTC;

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

int tmYear, tmMonth, tmDay, tmHour, tmMinute, tmSecond;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(200);

  bool parse = false;
  bool config = false;

  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    if (RTC.begin()) {
      RTC.adjust(DateTime(tmYear, tmMonth, tmDay, tmHour, tmMinute, tmSecond));
      config = true;
    }
  }

  if (parse && config) {
    Serial.print("DS3231 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS3231 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
}

void loop() {
}

bool getTime(const char *str) {
  if (sscanf(str, "%d:%d:%d", &tmHour, &tmMinute, &tmSecond) != 3) return false;
  return true;
}

bool getDate(const char *str) {
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;
  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tmDay = Day;
  tmMonth = monthIndex + 1;
  tmYear = Year;
  return true;
}
