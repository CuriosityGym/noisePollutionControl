#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

unsigned int UDPPortRemote = 4210;      // local port to listen on
unsigned int UDPPortLocal = 2392;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  replyBuffer[] = "acknowledged";       // a string to send back
String value="";
WiFiUDP Udp;
int led1 = 13;
int led2 = 2;
const char* ssid = "No Honking";
const char* password = "NoHonking";

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
unsigned int signalMax = 0;
unsigned int signalMin = 1024;
unsigned int peakToPeak = 0;   // peak-to-peak level
unsigned long loop_timer = 0;
String msg="";
int threshold = 75;
int amp=0; 
double maxm = 0.0;
double minm = 140.00;
unsigned long currentMillis1 = 0;
bool noiseLevel_1 = false;
bool sendMessage = false;
int count = 0;
int trafficNoise=0;
int a = 0;
bool measure = true;
// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024*2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.
#define MicPin A0 
#define VolumeGainFactorBits 0

void setup() {
     pinMode(led1, OUTPUT);
     pinMode(led2, OUTPUT);
    Serial.begin(115200);

      WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  /*
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Wait for WiFi");

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }*/
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());

    Udp.begin(UDPPortLocal);

    Udp.beginPacket("192.168.4.2", UDPPortRemote);//send ip to server
    char ipBuffer[255];
    WiFi.localIP().toString().toCharArray(ipBuffer, 255);
    Udp.write(ipBuffer);
    Udp.endPacket();
    Serial.println("Sent ip adress to server");
    }

void loop() {

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.print("Contents:");
    Serial.println(packetBuffer);
    msg = packetBuffer;
    String dBlimit = msg.substring(2);
    threshold = dBlimit.toInt();
  }
 /* for(int i=10; i<10;i++){
     trafficNoise += MeasureVolume();
   }*/
  if(measure == true){
     trafficNoise = MeasureVolume();
     a++;
  } 
  Serial.println(" Value: " + String(trafficNoise));
   if(trafficNoise > threshold)
    {
      Serial.println("Noise level 1");   
      noiseLevel_1 = true;
      //myservo.write(180);              // tell servo to go to position in variable 'pos'
      //delay(15);                       // waits 15ms for the servo to reach the position
      currentMillis1 = millis();
      count =0;
    }

 while(noiseLevel_1 == true && (millis() - currentMillis1) < 23000){
     for(int i=0; i<5;i++){
       trafficNoise += MeasureVolume();
     }     
      trafficNoise/= 5;
      Serial.print(" Value: " + String(trafficNoise));
      Serial.println("level1");
      if(trafficNoise > threshold){
        count++;
        delay(100);
      }
      trafficNoise= 0;
      if(count > 10){
       Serial.println("Noise level crossed"); 
       noiseLevel_1 = false; 
       currentMillis1 = millis(); 
       count =0; 
       delay(1000);
       sendMessage = true;
       a=0;
      }
   delay(50);   
  }
  if(sendMessage == true){
      digitalWrite(led1, HIGH);
         digitalWrite(led2, LOW);
      value = "1";
        Udp.beginPacket("192.168.4.2", UDPPortRemote);//send ip to server
    char ipBuffer[255];
    value.toCharArray(ipBuffer, 255);
    Udp.write(ipBuffer);
    Udp.endPacket();
    sendMessage = false; 
    Serial.println("message sent 1"); 
    a=0;
    delay(5000);
  }
  else{
    if(sendMessage == false && a > 50){
      value = "0";
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      Udp.beginPacket("192.168.4.2", UDPPortRemote);//send ip to server
    char ipBuffer[255];
    value.toCharArray(ipBuffer, 255);
    Udp.write(ipBuffer);
    Udp.endPacket();
    Serial.println("message sent 0");
    a = 0;
      //delay(1500);
         }  
     }

}


// calculate volume level of the signal and print to serial and LCD
int MeasureVolume()
{
  long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();
  //cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0; i < MicSamples; i++)
  {
//#ifdef ADCFlow
  //  while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
   // sbi(ADCSRA, ADIF); // restart adc
   // byte m = ADCL; // fetch adc data
   // byte j = ADCH;
   // int k = ((int)j << 8) | m; // form into an int
//#else
    int k = analogRead(MicPin);
//#endif
    int amp = abs(k - AmpMax);
    amp <<= VolumeGainFactorBits;
    //soundVolMax = max(soundVolMax, amp);
    if(soundVolMax > amp){
      soundVolMax = amp;
    }
    soundVolAvg += amp;
    soundVolRMS += ((long)amp*amp);
  }
  soundVolAvg /= MicSamples;
  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);
  //sei();

  float dB = 20.0*log10(soundVolRMSflt/AmpMax);
  double db = dB + 94 - 44 - 16;
  if(db > maxm){
    maxm = db;
  }
  if(db<minm){
    minm=db;
  }
  // convert from 0 to 100
  soundVolAvg = 100 * soundVolAvg / AmpMax; 
  soundVolMax = 100 * soundVolMax / AmpMax; 
  soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
  soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)
  int soundValues = abs(db);
  soundValues = map(soundValues, 0,36,0,120);
 /* 
  // print
  Serial.print("Time: " + String(millis() - t0));
  Serial.print(" Amp: Max: " + String(soundVolMax));
  Serial.print("% Avg: " + String(soundVolAvg));
  Serial.print("% RMS: " + String(soundVolRMS));
  Serial.print(" Max: " + String(maxm));
  Serial.print(" Min: " + String(minm));
  Serial.print(" Value: " + String(soundValues));
  Serial.println("% dB: " + String(db,3));*/
  
  return soundValues;
  /*
  if(dB >= limit){
     // if(xAngle >= -45 && xAngle <= 45){
     digitalWrite(led1, HIGH);
         digitalWrite(led2, LOW);
      value = "1";
        Udp.beginPacket("192.168.4.2", UDPPortRemote);//send ip to server
    char ipBuffer[255];
    value.toCharArray(ipBuffer, 255);
    Udp.write(ipBuffer);
    Udp.endPacket();
      delay(5000);
     }
  else{
      value = "0";
      digitalWrite(led1, LOW);
         digitalWrite(led2, HIGH);
        Udp.beginPacket("192.168.4.2", UDPPortRemote);//send ip to server
    char ipBuffer[255];
    value.toCharArray(ipBuffer, 255);
    Udp.write(ipBuffer);
    Udp.endPacket();
      delay(1500);
         //}  
     }

*/
}    
