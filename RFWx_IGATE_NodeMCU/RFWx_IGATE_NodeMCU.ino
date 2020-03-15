#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "RedmiGio_EXT";
const char* password = "maremmagatta";


const char CALL[]  = "IW5EJM-11"; //IGate's callsign
const char PSW[]  = "17853";      //APRS password for IGate's callsign

const char AprsServer[]  = "aprs.kcaprs.org";
const int dstPort  = 8080;
const int srcPort  = 4321;

WiFiUDP Udp;

char sentence[250];
char packetIgate[150];

void setup()
{
  Serial.begin(9600);
  Serial.println();
  WiFi.mode(WIFI_STA);
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

}


void loop()
{
  
  while (Serial.available()) {
     char aChar = Serial.read();
     if (aChar == '@') { 
        Serial.readBytesUntil('\n', packetIgate, 149);
        sprintf(sentence, "user %s pass %s vers RFWX MM igate 1.0\n%s", CALL, PSW, packetIgate);
        Serial.println(sentence);
        if(Udp.beginPacket(AprsServer, dstPort)) Serial.println("Server address resolved");
        Udp.write(sentence);
        if(Udp.endPacket()) Serial.println("Packet send!");
        memset(packetIgate, 0, sizeof(packetIgate));      
        delay(500);

          }      
     }     

  }
//char textToSend[] = "user IW5EJM-11 pass 17853 vers RFWX MM igate 1.0\nIW5EJM-7>APZMDM,WIDE1-1,WIDE2-2,IW5EJM-11:=4302.25N/01035.46E< IGATE test";
