#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

unsigned int UDPPort = 4210;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back
WiFiUDP Udp;

const char* ssid = "No Honking";
const char* password = "NoHonking";
String msg="";
int buzzer1 = 5;
int buzzer2 = 12;
int buzzer3 = 13;
int buzzer4 = 14;
void setup() {
  pinMode(buzzer1, OUTPUT);
  pinMode(buzzer2, OUTPUT);
  pinMode(buzzer3, OUTPUT);
  pinMode(buzzer4, OUTPUT);
    Serial.begin(115200);
  WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Wait for WiFi");
WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Wait for WiFi");

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
  Udp.begin(UDPPort);
  Serial.println();
    Serial.println("Started ap. Local ip: " + WiFi.localIP().toString());

 
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
   /* Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());*/

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    msg = packetBuffer;
    Serial.println("Contents:");
    Serial.println(msg);
    // send a reply, to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
   // Udp.endPacket();
 
   if (msg == "1") {
         digitalWrite(buzzer1, LOW);
         digitalWrite(buzzer2, LOW);
         digitalWrite(buzzer3, LOW);
         digitalWrite(buzzer4, LOW);
      }
      if (msg == "0") {
        digitalWrite(buzzer1, HIGH);
        digitalWrite(buzzer2, HIGH);
        digitalWrite(buzzer3, HIGH);
        digitalWrite(buzzer4, HIGH);
      }
  }

}
