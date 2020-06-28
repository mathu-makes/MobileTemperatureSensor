# MobileTemperatureSesnor
Arduino Uno controlled temperature relay with Fona 800 Cellular shield

Network connected Arduino Uno via Fona 800 with sim card; recieves and interprets text messages for remote control switching of a relay. 

The Uno understands commands via text such as "ON" "OFF" and many temperature combinations e.g. "ON 18 OFF 24". Such a command would instruct the Uno to switch the relay on if the DS18B20 temperature sensor was below or equal to 18 degrees and to turn if off again if the temperature was above or equal to 24 degrees. To display critical info such as on/off state and current temperature, a 16x2 LCD was implemented.
