#include <Wire.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <TimeLib.h>
#include <SparkFunBME280.h>
#include "CountDown.h"


//CHANGE THIS VARIABLES:
const char callsign[] = "IW5EJM";
const char ssid[] = "3";
const char tlm_callsign[] = "IW5EJM-3";
const char symbol[] = "_"; //select symbol from standard table, example Wx icon = _

//**** APRS COMMENT (max 43 chars)
const char APRS_CMNT[] = "RFWx MM DRA818 test";
const char longitude[] = "01036.59E";
const char latitude[] = "4303.51N";
const int altitude  = 200;

//APRS WX data tx interval
const int BeaconTime = 600;

//**** APRS_PRJ, Telemetry Project Title (max 23 chars)
const char APRS_PRJ[] = "MicroModem RFWx";
#define Voltage1 //uncomment this to enable telemetry for battery voltage
//#define Voltage2 //uncomment this variable to enable telemetry for secondary voltage reading

//no edit below this line

#define VoltDC1  A6 //battery voltage measurement pin input
#define VoltDC2  A7 //secondary voltage measurement pin input
#define R1R2     4 //voltage divider ratio, calculated as R2/(R1+R2)

#define SHORT_PAUSE 1000UL
#define LONG_PAUSE  30000UL
const char * OK_STR = "OK\r\n";
const char * MS_STR = "ms\r\n";

//port for modem communication
#define Serial2MM Serial

//port for DRA818 communication
#define DRA_TXD 3
#define DRA_RXD 2
SoftwareSerial dra_serial(DRA_TXD, DRA_RXD);

// Global sensor object
  BME280 mySensor;
  
//countdown to trigger data transmission
CountDown CD(CountDown::SECONDS);

int counter=0;
int counterTLM=0; 
char sentence[150];

const char preamble[] = "390";
const char txtail[] = "60";

//variables for voltage reading
int sensorDC1;
float VoltDCval1=0;
char VoltDCval_str1[10];

int sensorDC2;
float VoltDCval2=0;
char VoltDCval_str2[10];

typedef struct  {
  float temperatureC;
  float temperatureF;
  float pressure ;
  float humidity;
} WeatherStruct, *WeatherStructPtr;
WeatherStruct wx;    //declare la structure
WeatherStructPtr wx_ptr = &wx;

//function to send command to MM
boolean Serial2MMCommand(const char * command, const char * endMarker, unsigned long duration)
{
  Serial2MM.println(command);
  return waitForString(endMarker, duration);
}

//function that wait ok response from MM
boolean waitForString(const char * endMarker, unsigned long duration)
{
  int localBufferSize = strlen(endMarker); // we won't need an \0 at the end
  char localBuffer[localBufferSize];
  int index = 0;
  boolean endMarkerFound = false;
  unsigned long currentTime;

  memset(localBuffer, '\0', localBufferSize); // clear buffer

  currentTime = millis();
  while (millis() - currentTime <= duration) {
    if (Serial2MM.available() > 0) {
      if (index == localBufferSize) index = 0;
      localBuffer[index] = (uint8_t) Serial2MM.read();
      endMarkerFound = true;
      for (int i = 0; i < localBufferSize; i++) {
        if (localBuffer[(index + 1 + i) % localBufferSize] != endMarker[i]) {
          endMarkerFound = false;
          break;
        }
      }
      index++;
    }
    if (endMarkerFound) break;
  }
  return endMarkerFound;
}

//get rid of serial buffer
void emptySerial2MM_RX(unsigned long duration)
{
  unsigned long currentTime;
  currentTime = millis();
  while (millis() - currentTime <= duration) {
    if (Serial2MM.available() > 0) Serial2MM.read();
  }
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

//read WX data 
void getBmeValues(){
    
    float pres;

    wx.temperatureC = mySensor.readTempC();

    //*** calc standardized barometric pressure
    if ((pres=mySensor.readFloatPressure()) > 0.1f)
      wx.pressure = pres * ( pow(1.0 -(0.0065 * altitude * -1 /(273.15+wx.temperatureC)), 5.255));
    else
      wx.pressure = 0.0f;
        
    wx.temperatureF = mySensor.readTempF();
    wx.humidity =  mySensor.readFloatHumidity();
}


//send APRS packet
void Send2APRSWx()
{
  Serial2MM.begin(9600);  while (!Serial2MM);
  sprintf(sentence, "!=%s/%s_.../...g...t%03dr...p...P...h%02db%05d",                     latitude, 
                                                                                          longitude, 
                                                                                          (int)(wx.temperatureF), 
                                                                                          (int)(wx.humidity), 
                                                                                          (int)(wx.pressure/10));
  emptySerial2MM_RX(SHORT_PAUSE);
  if(Serial2MMCommand(sentence,"Packet sent", SHORT_PAUSE)) blink();
  delay(4500);
  Serial2MM.end();

 }
 

void Send2APRSPos()
{
  Serial2MM.begin(9600);  while (!Serial2MM);
  //send position packet
  sprintf(sentence, "!=%s/%s%s%s",latitude,longitude,symbol,APRS_CMNT);                                                    
  emptySerial2MM_RX(SHORT_PAUSE);
  if(Serial2MMCommand(sentence,"Packet sent", SHORT_PAUSE)) blink();
  delay(4500);
  Serial2MM.end();

}

void Send2APRSTlm()
{ 
  unsigned int rndnum=random(1000);  // random number here (telemetry packet ID)
  Serial2MM.begin(9600);  while (!Serial2MM);
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
  emptySerial2MM_RX(SHORT_PAUSE);
  if(Serial2MMCommand(sentence,"Packet sent", SHORT_PAUSE)) blink();
  delay(4500);
  
  counterTLM++;
  if (counterTLM >= 10) {
          //time to send telemetry labels and units
          counterTLM=0;
          //Define telemetry parameters (labels)
          sprintf(sentence, "!:%s :PARM.VBAT,V2", tlm_callsign);
          emptySerial2MM_RX(SHORT_PAUSE);
          if(Serial2MMCommand(sentence,"Packet sent", SHORT_PAUSE)) blink();
          delay(4500);

          //Define telemetry units
          sprintf(sentence, "!:%s :UNIT.V,V", tlm_callsign);
          emptySerial2MM_RX(SHORT_PAUSE);
          if(Serial2MMCommand(sentence,"Packet sent", SHORT_PAUSE)) blink();
          delay(4500);
          
          //Add telemetry coefficient so the APRS protocol can convert your raw values into real value.
          sprintf(sentence, "!:%s :EQNS.0,1,0,0,1,0,0,0,0,0,0,0,0,0,0", tlm_callsign);
          if(Serial2MMCommand(sentence,"Packet sent", SHORT_PAUSE)) blink();
          delay(4500);
          }
  Serial2MM.end();
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  //init DRA818 RTX MODULE
  dra_serial.begin(9600); while (!dra_serial); //serial to DRA818 init
  sprintf(sentence, "AT+DMOSETGROUP=0,144.8000,144.8000,0000,1,0000");                                                    
  dra_serial.println(sentence);
  delay(450);

  sprintf(sentence, "AT+DMOSETVOLUME=8");                                                    
  dra_serial.println(sentence);
  delay(450);  

  sprintf(sentence, "AT+SETFILTER=1,1,1");                                                    
  dra_serial.println(sentence);
  delay(450); 
  dra_serial.end();


  
  //init BME280
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;
  mySensor.settings.runMode = 3; //Normal mode
  mySensor.settings.tStandby = 0;
  mySensor.settings.filter = 0;
  mySensor.settings.tempOverSample = 1;
  mySensor.settings.pressOverSample = 1;
  mySensor.settings.humidOverSample = 1;
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  mySensor.begin();
  blink();

  //init modem
  delay(1000);
  Serial2MM.begin(9600); while (!Serial2MM);
  emptySerial2MM_RX(SHORT_PAUSE);
  blink();
  delay(150);

  sprintf(sentence, "c%s",callsign);
  Serial2MM.println(sentence);                                                   
  delay(450);

  sprintf(sentence, "sc%s",ssid);                                                    
  Serial2MM.println(sentence);
  delay(450);
  
  sprintf(sentence, "w%s",preamble);                                                    
  Serial2MM.println(sentence);  
  delay(450);
  
  sprintf(sentence, "W%s",txtail);                                                    
  Serial2MM.println(sentence);  
  delay(450);
    
  sprintf(sentence, "S");                                                    
  if(Serial2MMCommand(sentence,"Configuration saved", SHORT_PAUSE)) blink();
  delay(450);
  
  CD.stop();
  delay(1000);
  Serial2MM.end();

}

void loop() {
  
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(50); 
  digitalWrite(LED_BUILTIN, LOW);
  
  if (CD.remaining() < 1) { 
    CD.stop();
    getBmeValues();
    digitalWrite(LED_BUILTIN, HIGH);
    Send2APRSWx();
    counter++;
    digitalWrite(LED_BUILTIN, LOW);
    if (counter == 3) {
        digitalWrite(LED_BUILTIN, HIGH);
        Send2APRSPos();
        digitalWrite(LED_BUILTIN, LOW);
         }
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
    
  
  delay(1000);

}
