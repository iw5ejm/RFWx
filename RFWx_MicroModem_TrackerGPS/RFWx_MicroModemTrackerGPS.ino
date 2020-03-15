#include <stdlib.h>
#include <SparkFunBME280.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "CountDown.h"

//CHANGE THIS VARIABLES:
const char callsign[] = "IW5EJM";
const char ssid[] = "9";
const char tlm_callsign[] = "IW5EJM-9";
const char symbol[] = "("; //select symbol from standard table, example Wx icon = _ http://www.aprs.org/symbols.html

//**** APRS COMMENT (max 43 chars)
const char APRS_CMNT[] = " RFWx MM DRA818 tracker test";

//APRS tracker data tx interval (seconds)
const int BeaconTime = 180;

//**** APRS_PRJ, Telemetry Project Title (max 23 chars)
const char APRS_PRJ[] = "MicroModem RFWx";
//#define Voltage1 //uncomment this line to enable telemetry for battery voltage
//#define Voltage2 //uncomment this line to enable telemetry for secondary voltage reading



#define LED_FIX A5 //led on when gps receiver is correctly fixed
#define LED_TX A4  //led on when transmitting packet
#define BUT A3     //pin to connect button to trigger manual packet transmission: ground to tx

//no edit below this line

#define VoltDC1  A6 //battery voltage measurement pin input
#define VoltDC2  A7 //secondary voltage measurement pin input
#define R1R2     4 //voltage divider ratio, calcuLated as R2/(R1+R2)

//port for GPS communication
#define GPSrx A0      //tx pin into RX GPS connection
#define GPStx A1      //rx pin into TX GPS connection
SoftwareSerial GPSSerial(GPStx, GPSrx);  // Serial port for GPS receiver
Adafruit_GPS GPS(&GPSSerial);

CountDown CD(CountDown::SECONDS); //countdown to trigger data transmission

const char preamble[] = "450";
const char txtail[] = "90";
int counter=0;
int counterTLM=0; 
char sentence[150];
//String buf;
unsigned long secondtimer = 0;
unsigned long timer = 0;
//port for modem communication
#define Serial2MM Serial


//variables for voltage reading
int sensorDC1;
float VoltDCval1=0;
char VoltDCval_str1[10];

int sensorDC2;
float VoltDCval2=0;
char VoltDCval_str2[10];


void Send2APRSTrk()
{ 
  if (GPS.fix) 
    { digitalWrite(LED_TX, HIGH);

      
      String buf;

      Serial2MM.print("lla");
      Serial2MM.print(GPS.latitude, 2);
      Serial2MM.println(GPS.lat);
      delay(450);

      Serial2MM.print("llo");
      if (GPS.longitude < 10000) Serial2MM.print("0");
      Serial2MM.print(GPS.longitude, 2);
      Serial2MM.println(GPS.lon);
      delay(450);

      
      //send position packet

      Serial2MM.print("@");
      Serial2MM.print(GPS.angle, 0);
      Serial2MM.print("/");
      Serial2MM.print(GPS.speed, 0);
      Serial2MM.println(APRS_CMNT);
      
      digitalWrite(LED_TX, LOW);

      }
  delay(4500);
}


void Send2APRSTlm()
{ 
  digitalWrite(LED_TX, HIGH);
  unsigned int rndnum=random(1000);  // random number here (telemetry packet ID)

  #ifdef Voltage1
  //read telemetry data
  sensorDC1=analogRead(VoltDC1);
  VoltDCval1=(5.0/1023) * R1R2 * sensorDC1;
  #endif


  #ifdef Voltage2
  sensorDC2=analogRead(VoltDC2);
  VoltDCval2=(5.0/1023) * R1R2 * sensorDC2;
  #endif

  //send telemetry data
  sprintf(sentence, "!T#%03d,%03d,%03d,000,000,000,00000000",               rndnum, 
                                                                            VoltDCval1, 
                                                                            VoltDCval2);
  Serial2MM.println(sentence);
  delay(4500);
  
  counterTLM++;
  if (counterTLM >= 10) {
          //time to send telemetry labels and units
          counterTLM=0;
          //Define telemetry parameters (labels)
          sprintf(sentence, "!:%s:PARM.VBAT,V2", tlm_callsign);
          Serial2MM.println(sentence);
          delay(4500);

          //Define telemetry units
          sprintf(sentence, "!:%s:UNIT.V,V", tlm_callsign);
          Serial2MM.println(sentence);
          delay(4500);
          
          //Add telemetry coefficient so the APRS protocol can convert your raw values into real value.
          sprintf(sentence, "!:%s:EQNS.0,1,0,0,1,0,0,0,0,0,0,0,0,0,0", tlm_callsign);
          Serial2MM.println(sentence);
          delay(4500);
          }
  digitalWrite(LED_TX, LOW);
}

void blink()
{
  boolean led = false;
  led = digitalRead(LED_BUILTIN);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);  
  delay(10);
  digitalWrite(LED_BUILTIN, led);  
}

void setup() {
  pinMode(LED_FIX, OUTPUT);
  pinMode(LED_TX, OUTPUT);
  pinMode(BUT, INPUT_PULLUP);

  digitalWrite(LED_TX, LOW);
  digitalWrite(LED_FIX, HIGH); 
  delay(50); 
  digitalWrite(LED_FIX, LOW);
  
  delay(1000); //wait for micromodem to correctly bootup

  Serial2MM.begin(9600); while (!Serial2MM); //serial modem init
  blink();
  delay(450);
  
  sprintf(sentence, "c%s",callsign);                                                    
  Serial2MM.println(sentence);
  delay(450);
  
  sprintf(sentence, "sc%s",ssid);                                                    
  Serial2MM.println(sentence);
  delay(450);

  sprintf(sentence, "ls%s",symbol);                                                    
  Serial2MM.println(sentence);
  delay(450);
  
  sprintf(sentence, "w%s",preamble);                                                    
  Serial2MM.println(sentence);
  delay(450);
  
  sprintf(sentence, "W%s",txtail);                                                    
  Serial2MM.println(sentence);
  delay(450);
  
    
  sprintf(sentence, "S");                                                    
  Serial2MM.println(sentence);
  delay(450);
  
  Serial2MM.flush();
  delay(450);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Sets output to only RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Sets the output to 1/second. If you want you can go higher/lower
  CD.stop();

  delay(1000);

}

void loop() {
  
char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }   

  digitalWrite(LED_FIX, GPS.fix);
  if (CD.remaining() < 1 | !digitalRead(BUT)) { 
    CD.stop();
    Send2APRSTrk();
    counter++;
     if (counter >= 5) {
        counter=0;
        #if defined (Voltage1) || defined (Voltage2)       
        digitalWrite(LED_BUILTIN, HIGH);
        Send2APRSTlm();
        digitalWrite(LED_BUILTIN, LOW);
        #endif

         }  
     CD.start(BeaconTime);
     }
    
}
