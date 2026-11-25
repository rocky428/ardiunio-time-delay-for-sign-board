# ardiunio-time-delay-for-sign-board
time-delay for sign board for auto on,off at mornig and evening -Based on TimeAlarmExample from TimeAlarms library created by Michael Margolis
1. connect ardunio and rtc, set time using the set_current_time_in_rtc.ino code. just upload one time code to ardunio. When the rtc battery removed while it's connected to ardunio, you should upload the code to ardunio after battery connected to rtc. otherwice it cann't identify current time.

2. Upload updated new_relay.ino code. 

first upload time_set_ic_RTC_DS3231.ino
and again upload code but comment out this part "//rtc.adjust(DateTime(2026, 11, 25, 20, 52, 0));" so every time the board resets (an upload, power on, or the reset button), the RTC is did not go back to 12/11/2026 20:52:00.

second upload auto_Sunrise_relay_time.ino
