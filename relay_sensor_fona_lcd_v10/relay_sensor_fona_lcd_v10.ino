/* LCD library */

#include <LiquidCrystal.h>

/*Fona libraries*/
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#define FONA_RI 5

#define LED 8 // LED connected to pin 11

/* DS18B20 Temperature Sensor libraries */
#include <OneWire.h>
#include <DallasTemperature.h>

/* Temperatire initialisers */
int reading = 0;
int vTemp_on = 0;
int vTemp_off = 0;
int read_on = 15;
int read_off = 30;
int timed = 45;

/* LCD PINOUTS & INT */
LiquidCrystal lcd(9, 10, A5, A4, A3, A2);

int lcd_key     = 0;
int adc_key_in  = 0;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int read_LCD_buttons(){               // read the buttons
    adc_key_in = analogRead(0);       // read the value from the sensor 

     delay(85); //switch debounce delay. Increase this delay if incorrect switch selections are returned.
     int k = (analogRead(0) - adc_key_in); //gives the button a slight range to allow for a little contact resistance noise
    if (5 < abs(k)) return btnNONE;  // double checks the keypress. If the two readings are not equal +/-k value after debounce delay, it tries again.
 
    // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    // We make this the 1st option for speed reasons since it will be the most likely result
 
    if (adc_key_in > 1000) return btnNONE; 
 
    // For V1.1 us this threshold
    if (adc_key_in < 50)   return btnRIGHT;  
    if (adc_key_in < 250)  return btnUP; 
    if (adc_key_in < 450)  return btnDOWN; 
    if (adc_key_in < 650)  return btnLEFT; 
    if (adc_key_in < 850)  return btnSELECT;
 
    return btnNONE;                // when all others fail, return this.
}

// Data wire is plugged into pin 7 on the Arduino
#define ONE_WIRE_BUS 7

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// 1-Wire temp sensors :: Addresses
DeviceAddress insideThermometer = { 0x28, 0xB1, 0x67, 0x3C, 0x07, 0x00, 0x00, 0x4E };

/*-----( Relay Declarations )-----*/
#define RELAY_ON 0
#define RELAY_OFF 1
/*-----( Declare objects )-----*/
/*-----( Declare Variables )-----*/
#define Relay_1  6  // Arduino Digital I/O pin number
/*#define Relay_2  7*/

/*-----Busy Wait Loop defintion-----*/

#define BUSYWAIT 5000  // milliseconds

// this is a large buffer for replies
char replybuffer[255];

// or comment this out & use a hardware serial port like Serial1 (see below)
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

boolean fonainit(void) {
  Serial.println(F("Initializing....(May take 3 seconds)"));
   
  // make it slow so its easy to read!
  fonaSS.begin(4800); // if you're using software serial
  //Serial1.begin(4800); // if you're using hardware serial

  // See if the FONA is responding
  if (! fona.begin(fonaSS)) {           // can also try fona.begin(Serial1) 
    Serial.println(F("Couldn't find FONA"));
    return false;
  }
  Serial.println(F("FONA is OK"));
  return true;
  
}

void setup() {
  
  // start serial port
  Serial.begin(115200);

  Serial.println("::Mobile Thermostat & Temperature Sensor::");
  Serial.println("");
  delay(3000);

  /* LCD */
  lcd.begin(16, 2);               // start the library
  lcd.setCursor(0,0);             // set the LCD cursor position 
  lcd.print(" AM Thermostats");  // print a simple message on the LCD
  lcd.setCursor(0,1);
  lcd.print("    WELCOME    ");
  
  /* LED */
  pinMode(LED, OUTPUT);

  /* Fona */
  Serial.println(F("FONA basic test"));

  while (! fonainit()) {
    delay(5000);
  }
  
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }
  
  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_RI, HIGH); // turn on pullup on RI
  // turn on RI pin change on incoming SMS!
  fona.sendCheckReply(F("AT+CFGRI=1"), F("OK"));

/*-------------------------------------------------------------*/

  /* DS18B20 Temperature Sensor setup */
  
  // Start up the library
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(insideThermometer, 10);

/*-------------------------------------------------------------*/

  /* Relays */

  //-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_1, RELAY_OFF);
  /*digitalWrite(Relay_2, RELAY_OFF);*/

  while (! Serial); // Wait until Serial is ready
  
  //---( THEN set pins as outputs )----  
  pinMode(Relay_1, OUTPUT);
  /*pinMode(Relay_2, OUTPUT);*/
  delay(4000); //Check that all relays are inactive at Reset
  Serial.println("");
  Serial.println("Ready to start");
  Serial.println("");

/*-------------------------------------------------------------*/
  /* Temperature startup menu */ 
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print(" AMT");

  uint16_t vbat;
  fona.getBattPercent(&vbat);
  lcd.setCursor(10,0);
  lcd.print(F(" b")); lcd.print(vbat); lcd.print(F("%"));
  
  lcd.setCursor(0,1);
  lcd.print("                ");
  
  while ((millis()/1000) < timed) {
    unsigned long time;
    
    /*if ((millis()/1000) < timed) {*/
    int outta_time = timed - millis()/1000;
    Serial.println(outta_time);
    lcd.setCursor (13,1);
    lcd.print ("   ");  // Three spaces to clear the area of screen the number will occupy
    lcd.setCursor (13,1);
    lcd.print("m");
    lcd.print (outta_time);
    /*}*/
    
    /*lcd.setCursor(10,1);
    float insideTempC = sensors.getTempC(insideThermometer);
    lcd.print(insideTempC);
    lcd.print("C");*/
    
    lcd.setCursor(0,1);             // move to the begining of the second line
    lcd_key = read_LCD_buttons();   // read the buttons
  
    switch (lcd_key){               // depending on which button was pushed, we perform an action
      
      case btnSELECT:{
        lcd.print("SELECT!");  //  push button "SELECT" and show the word on the screen
        timed = 0;
        break;
      }
      case btnNONE:{
        lcd.setCursor(0,1);
        lcd.print("*");
        lcd.print(read_on);
        lcd.setCursor(3,1);
        lcd.print("^");
        lcd.print(read_off);  //  No action  will show "None" on the screen
        /*timer = 1;*/
        break;
      }
      case btnLEFT: {
        read_on = --read_on;
        if (read_on < 15) {
          read_on = 15;
        }
        lcd.setCursor(1,1);
        lcd.print(read_on);
        break;
      }
      case btnRIGHT: {
        read_on = ++read_on;
        if (read_on > 30) {
          read_on = 30;
        }
        lcd.setCursor(1,1);
        lcd.print(read_on);
        break;
      }
      case btnUP: {
        read_off = ++read_off;
        if (read_off > 30) {
          read_off = 30;
        }
        lcd.setCursor(4,1);
        lcd.print(read_off);
        break; 
      }
      case btnDOWN: {
        read_off = --read_off;
        if (read_off < 15) {
          read_off = 15;
        }
        lcd.setCursor(4,1);
        lcd.print(read_off);
        break;
      }  
    }
    if (read_on < 15) {
      read_on = 15;
    }
    if (read_on > 30) {
      read_on = 30;
    }
    if (read_off < 15) {
      read_on = 15;
    }
    if (read_off > 30) {
      read_off = 30;
    }
    if (read_off <= read_on) {
      read_off = read_on + 1;
      if (read_off > 30) {
        read_on = read_on - 1; 
      }   
    }
  }
    
  vTemp_on = read_on;
  vTemp_off = read_off;
    
  if (vTemp_on < 15) {
    vTemp_on = 15;
  }
  if (vTemp_on > 30) {
    vTemp_on = 30;
  }
  if (vTemp_off < 15) {
    vTemp_on = 15;
  }
  if (vTemp_off > 30) {
    vTemp_off = 30;
  }
  /*if (vTemp_on > vTemp_off) {
    vTemp_on = vTemp_off - 1;
  }*/
  if (vTemp_off < vTemp_on) {
    vTemp_off = vTemp_on + 1; 
  }

  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(1,1);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(2,1);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(1,1);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(2,1);
  lcd.print(".");
  delay(1000);
  
}//--(end setup)--\\

/* Temperature Data parse */
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
  } else {
    Serial.print("C: ");
    Serial.print(tempC);
    Serial.print(" F: ");
    Serial.print(DallasTemperature::toFahrenheit(tempC));
  }

/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
}

/* Fona message # initialiser */
int8_t lastsmsnum = 0;

void loop(void)
{  
  Serial.println("::New-Loop::");
  Serial.println("");
  
  /*delay(2000);*/
/*-------------------------------------------------------------*/
/* LCD */
  lcd.setCursor(9,1);             // move cursor to second line "1" and 9 spaces over
  /*lcd.print(millis()/1000);       // display seconds elapsed since power-up*/
 
  lcd.setCursor(0,1);             // move to the begining of the second line
  lcd_key = read_LCD_buttons();   // read the buttons
/*-------------------------------------------------------------*/

  uint16_t vbat;
  fona.getBattPercent(&vbat);
  lcd.setCursor(10,0);
  lcd.print(F(" b")); lcd.print(vbat); lcd.print(F("%"));
  
  /*while (fona.getNetworkStatus() != 1) {
    Serial.println("Waiting for cell connection");
    delay(2000);
  }*/
  
  // this is a 'busy wait' loop, we check if the interrupt
  // pin went low, and after BUSYWAIT milliseconds break out to check
  // manually for SMS's and connection status
  // This would be a good place to 'sleep'
  for (uint16_t i=0; i<BUSYWAIT; i++) {
     if (! digitalRead(FONA_RI)) {
        // RI pin went low, SMS received?
        Serial.println(F("RI went low"));
        break;
     } 
     delay(1);
  }
  
  int8_t smsnum = fona.getNumSMS();
  if (smsnum < 0) {
    Serial.println(F("Could not read # SMS"));
    return;
  } else {
    Serial.print(smsnum);
    Serial.println(F(" SMS's on SIM card!"));
  }
  
  if (smsnum == 0) return;
  /* SMS - n = 0 */
  
  // there's an SMS!
  uint8_t n; 
  /*Serial.println(n);*/ /*n = 0 */
  int v;
  for (n = 0; n != smsnum; v = ++n);
  int nxt = -1 + v;
  /*Serial.println(n);
  Serial.println(v);
  Serial.println(nxt);
  Serial.println(smsnum);*/
  Serial.print(v);
  Serial.println(" Messages recognised by AM Thermostat");
 
  /*if (smsnum != 1 && n != 1 && v != 1)*/
  
  while (true) {
    uint16_t smslen;
    char sender[25];
    
    Serial.print(F("\n\rReading SMS #")); Serial.println(n);
    uint8_t len = fona.readSMS(n, replybuffer, 250, &smslen); // pass in buffer and max len!
    // if the length is zero, its a special case where the index number is higher
    // so increase the max we'll look at!
    if (len == 0) {
      Serial.println(F("[empty slot]"));
      n++;
      continue;
    } 
      
    if (! fona.getSMSSender(n, sender, sizeof(sender))) {
      // failed to get the sender?
      sender[0] = 0;
    }
    
    Serial.println(" ");
    Serial.print(F("***** SMS #")); Serial.print(n);
    Serial.print(" ("); Serial.print(len); Serial.println(F(") bytes *****"));
    Serial.println(replybuffer);
    Serial.print(F("From: ")); Serial.println(sender);
    Serial.println(F("*****"));
      
    if (strcasecmp(replybuffer, "off") == 0) {
      digitalWrite(Relay_1, RELAY_OFF);
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      Serial.println("::GLOBAL OFF::");
      lcd.print("                ");
      lcd.setCursor(5,0);
      lcd.print("-OFF");
      float insideTempC = sensors.getTempC(insideThermometer);
      lcd.setCursor(9,1);
      lcd.print(insideTempC);
      lcd.print("\337C");

      /*delay(3000);*/
      Serial.println(" ");
      
      Serial.print("Getting temperatures...\n\r");
      sensors.requestTemperatures();
  
      Serial.print("Inside temperature is: ");
      lcd.print("Inside temperature is: ");
      printTemperature(insideThermometer);
      Serial.print("\n\r");
      
      Serial.println(" ");
      /*delay(3000);*/
      return;
    }
      
    if (
      strcasecmp(replybuffer, "on")== 0 ||
      
      strcasecmp(replybuffer, "on 15")== 0 ||

      strcasecmp(replybuffer, "on 16")== 0 ||

      strcasecmp(replybuffer, "on 17")== 0 ||

      strcasecmp(replybuffer, "on 18")== 0 ||

      strcasecmp(replybuffer, "on 19")== 0 ||

      strcasecmp(replybuffer, "on 20")== 0 ||

      strcasecmp(replybuffer, "on 21")== 0 ||

      strcasecmp(replybuffer, "on 22")== 0 ||

      strcasecmp(replybuffer, "on 23")== 0 ||

      strcasecmp(replybuffer, "on 24")== 0 ||
      
      strcasecmp(replybuffer, "on 25")== 0 ||

      strcasecmp(replybuffer, "manual") == 0
    ) {
      Serial.println("::GLOBAL ON::");
      lcd.setCursor(0,1);
      lcd.print("                ");
      lcd.setCursor(5.,0);
      lcd.print(" +ON");
      float insideTempC = sensors.getTempC(insideThermometer);
      lcd.setCursor(9,1);
      lcd.print(insideTempC);
      lcd.print("\337C");
      break;
    }
  }

  if (strcasecmp(replybuffer, "on") == 0) {
    vTemp_on = 18;
    vTemp_off = 21;
    lcd.setCursor(0,0);
    lcd.print("i");
  }
  
  if (strcasecmp(replybuffer, "on 15") == 0) {
    vTemp_on = 15;
    vTemp_off = 20; 
    lcd.setCursor(0,0);
    lcd.print("i");  
  }

  if (strcasecmp(replybuffer, "on 16") == 0) {
    vTemp_on = 16;
    vTemp_off = 21;
    lcd.setCursor(0,0);
    lcd.print("i");  
  }

  if (strcasecmp(replybuffer, "on 17") == 0) {
    vTemp_on = 17;
    vTemp_off = 22;
    lcd.setCursor(0,0);
    lcd.print("i");   
  }

  if (strcasecmp(replybuffer, "on 18") == 0) {
    vTemp_on = 18;
    vTemp_off = 23;
    lcd.setCursor(0,0);
    lcd.print("i");   
  }

  if (strcasecmp(replybuffer, "on 19") == 0) {
    vTemp_on = 19;
    vTemp_off = 24;
    lcd.setCursor(0,0);
    lcd.print("i"); 
  }

  if (strcasecmp(replybuffer, "on 20") == 0) {
    vTemp_on = 20;
    vTemp_off = 25;
    lcd.setCursor(0,0);
    lcd.print("i");  
  }

  if (strcasecmp(replybuffer, "on 21") == 0) {
    vTemp_on = 21;
    vTemp_off = 26;
    lcd.setCursor(0,0);
    lcd.print("i");   
  }

  if (strcasecmp(replybuffer, "on 22") == 0) {
    vTemp_on = 22;
    vTemp_off = 27;
    lcd.setCursor(0,0);
    lcd.print("i");   
  }

  if (strcasecmp(replybuffer, "on 23") == 0) {
    vTemp_on = 23;
    vTemp_off = 28;
    lcd.setCursor(0,0);
    lcd.print("i");   
  }

  if (strcasecmp(replybuffer, "on 24") == 0) {
    vTemp_on = 24;
    vTemp_off = 29;
    lcd.setCursor(0,0);
    lcd.print("i");  
  }

  if (strcasecmp(replybuffer, "on 25") == 0) {
    vTemp_on = 25;
    vTemp_off = 30;
    lcd.setCursor(0,0);
    lcd.print("i");   
  }

  if (strcasecmp(replybuffer, "manual") == 0) {
    /*vTemp_on = 15;
    vTemp_off = 30;*/
    lcd.setCursor(0,0);
    lcd.print("-");
  }
  
  /*delay(1000);*/

  int vCounter = 0;
  while (vCounter < 60) { /* legacy setting : 6 */
    lcd.setCursor(8,1);
    lcd.print("~");

    /*read_on = vTemp_on;
    read_off = vTemp_off;*/

    lcd_key = read_LCD_buttons();   // read the buttons
    switch (lcd_key){               // depending on which button was pushed, we perform an action
      /*case btnNONE:{
        lcd.setCursor(0,1);
        lcd.print("*");
        lcd.print(vTemp_on);
        lcd.setCursor(3,1);
        lcd.print("^");
        lcd.print(vTemp_off);  //  No action  will show "None" on the screen
        break;
      }*/
      case btnLEFT: {
        vTemp_on = --vTemp_on;
        if (vTemp_on < 15) {
          vTemp_on = 15;
        }
        lcd.setCursor(1,1);
        lcd.print(vTemp_on);
        break;
      }
      case btnRIGHT: {
        vTemp_on = ++vTemp_on;
        if (vTemp_on > 30) {
          read_on = 30;
        }
        lcd.setCursor(1,1);
        lcd.print(vTemp_on);
        break;
      }
      case btnUP: {
        vTemp_off = ++vTemp_off;
        if (vTemp_off > 30) {
          vTemp_off = 30;
        }
        lcd.setCursor(4,1);
        lcd.print(vTemp_off);
        break; 
      }
      case btnDOWN: {
        vTemp_off = --vTemp_off;
        if (vTemp_off < 15) {
          vTemp_off = 15;
        }
        lcd.setCursor(4,1);
        lcd.print(vTemp_off);
        break;
      }  
    }
    
    if (vTemp_on < 15) {
      vTemp_on = 15;
    }
    if (vTemp_on > 30) {
      vTemp_on = 30;
    }
    if (vTemp_off < 15) {
      vTemp_on = 15;
    }
    if (vTemp_off > 30) {
      vTemp_off = 30;
    }
    if (vTemp_off <= vTemp_on) {
      vTemp_off = vTemp_on + 1;
      if (vTemp_off > 30) {
        vTemp_on = vTemp_on - 1; 
      }
    }

    lcd.setCursor(0,1);
    lcd.print("*");
    lcd.print(vTemp_on);
    lcd.setCursor(3,1);
    lcd.print("^");
    lcd.print(vTemp_off);
    
    /*digitalWrite(Relay_2, RELAY_ON);// set the Relay ON*/
    /*delay(100);*/              // wait for a second
    
    Serial.print(vCounter);
    Serial.println(" vCounter value");

    Serial.println(" ");
    
    Serial.print("Getting temperatures...\n\r");
    sensors.requestTemperatures();
  
    Serial.print("Inside temperature is: ");
    printTemperature(insideThermometer);
    Serial.print("\n\r");
    
    float insideTempC = sensors.getTempC(insideThermometer);
    Serial.println(" ");
    Serial.print(vTemp_on);
    Serial.println(" C minimum temperature");
    Serial.print(vTemp_off);
    Serial.println(" C maximum temperature");
    Serial.println(" ");
    
    if (insideTempC < vTemp_on){
      digitalWrite(Relay_1, RELAY_ON);
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      Serial.println("Relay 1 ON");
        
      /*delay(2000);*/
      Serial.print("Getting temperatures...\n\r");
      sensors.requestTemperatures();
    
      Serial.print("Inside temperature is: ");
      printTemperature(insideThermometer);
      Serial.print("\n\r");

      lcd.setCursor(9,1);
      lcd.print(insideTempC);
      lcd.print("\337C");
    }

    lcd.setCursor(9,1);
    lcd.print(insideTempC);
    lcd.print("\337C");
    
    if (insideTempC > vTemp_off){
      digitalWrite(Relay_1, RELAY_OFF);
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      Serial.println("Relay 1 OFF");
      /*lcd.print("Temperature over ");*/
      
      /*delay(2000);*/
      Serial.print("Getting temperatures...\n\r");
      sensors.requestTemperatures();
    
      Serial.print("Inside temperature is: ");
      printTemperature(insideThermometer);
      Serial.print("\n\r");

      lcd.setCursor(9,1);
      lcd.print(insideTempC);
      lcd.print("\337C");
    }

    lcd.setCursor(9,1);
    lcd.print(insideTempC);
    lcd.print("\337C");
    
    /*delay(3000);*/
    Serial.println("");
    vCounter++;

    lcd.setCursor(8,1);
    lcd.print(" ");
  }
  Serial.println("::Finished-Loop::");
  /*digitalWrite(Relay_2, RELAY_OFF);// set the Relay OFF*/
  /*delay(100);  */            // wait for a second
  Serial.println("");
  /*delay(3000);*/
}
