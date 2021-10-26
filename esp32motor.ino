#include <TelnetStream.h>
#include <TelnetPrint.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <FastLED.h>
#include <Smoothed.h>   // Include the library

Smoothed <int> smoothedCad;
Smoothed <int> smoothedOut; 

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const char* sw_version = "20210816";
const char* host = "esp32";
const char* ssid = "esp32Remote";
const char* password = "123456";
AsyncWebServer server(80);

#define NUM_LEDS 1
#define DATA_PIN 27

// Define the array of leds
CRGB leds[NUM_LEDS];

 //analogIn
int gach_PIN= 33;
int pot_PIN = 32;
//digitalIn
int cad_PIN = 36;
int vit_PIN = 19;
int add_vit = 18;
//analogOut
int mOut = 25;

volatile unsigned long interrupt_vit,interrupt_vit_old;
volatile unsigned long interrupt_cad,interrupt_cad_old;
unsigned long old_time = 0;

unsigned long s100_time = 0, s250_time = 0, s1000_time = 0;

volatile unsigned long passed = 1000;
volatile unsigned long passed2 = 0;
volatile unsigned int cadence=0, old_cadence=0;
long gachValue =0,potValue = 0;

unsigned long time_now = 0, interval= 0;
volatile float vitesse = 0,old_vitesse = 0;
int dacVoltage = 0;

int aimants=12;
int diametre=780;
int minMotor = 30;
int maxMotor = 255;
float frac = 0.9;
int minCadence = 10;
int maxCadence = 100;


void printStream(String message){
  Serial.print(message);
  TelnetStream.print(message);
}
void printStreamln(String message){
  Serial.println(message);
  TelnetStream.println(message);
}

void IRAM_ATTR isr_cad(){
  portENTER_CRITICAL_ISR(&mux);
    interrupt_cad = millis();
    passed = interrupt_cad - interrupt_cad_old;
    cadence = 60/aimants*1000/passed;
    //cadence = int(frac*float(old_cadence) + (1-frac)*float(cadence));
    smoothedCad.add(cadence);
    //old_cadence = cadence;
    interrupt_cad_old = interrupt_cad;
  portEXIT_CRITICAL_ISR(&mux); 
}

void IRAM_ATTR isr_vit(){
  portENTER_CRITICAL_ISR(&mux);
    if (millis() -interrupt_vit>100){
      interrupt_vit = millis();
      passed2 = interrupt_vit - interrupt_vit_old;
      vitesse = frac * (diametre * 3.141528 * 3.6 / (float(passed2)))+ (1-frac)* old_vitesse; 
      interrupt_vit_old = interrupt_vit;
      old_vitesse = vitesse;
    }
    
  portEXIT_CRITICAL_ISR(&mux);
}

void wifiAPSetup(){
  WiFi.softAP(ssid, password);
  //printStreamln();
  printStream("software version: ");
  printStreamln(sw_version);
  printStream("host: ");
  printStreamln(host);
  printStream("ssid: ");
  printStreamln(ssid);
  printStream("password: ");
  printStreamln(password);
  IPAddress IP = WiFi.softAPIP();
  printStream("url: http://");
  printStream(String(IP));
  printStreamln("/update");
 
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    printStreamln("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  printStreamln("mDNS responder started");
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "LiftController Version: ");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  printStreamln("HTTP server started");
  TelnetStream.begin();
}


void setup() {
  dacWrite(mOut,0);
  analogReadResolution(10);
  pinMode(cad_PIN,INPUT_PULLUP);
  pinMode(vit_PIN,INPUT_PULLUP);
  pinMode(add_vit,OUTPUT);
  digitalWrite(add_vit,HIGH);
  Serial.begin(115200);
  wifiAPSetup();
  FastLED.addLeds<SK6812, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
  leds[0] = 0x00f000;
  FastLED.show();
  attachInterrupt(digitalPinToInterrupt(cad_PIN), isr_cad, RISING);
  attachInterrupt(digitalPinToInterrupt(vit_PIN), isr_vit, FALLING);
  
  smoothedCad.begin(SMOOTHED_EXPONENTIAL,20);
  smoothedOut.begin(SMOOTHED_EXPONENTIAL,20);
}

void sendToMotor(){
  int cad = smoothedCad.get();
  if (cadence > minCadence){
      if (gachValue > potValue){
        dacVoltage = map(gachValue,0,100,minMotor,maxMotor);
      } else {
        if (potValue == 0){
          dacVoltage = map(cadence,minCadence, maxCadence, minMotor,maxMotor);
        } else {
          dacVoltage = map(potValue,0,100,minMotor,maxMotor);
        }
      }  
    dacVoltage = map(cadence,minCadence,maxCadence,minMotor,maxMotor);
  } else {
    dacVoltage =  0;
  }
  
  if (vitesse > 25) {
    dacVoltage = 0;
  }
  smoothedOut.add(dacVoltage);
  //analogWrite(mOut,dacVoltage);
  dacWrite(mOut,smoothedOut.get());
}
void s100_loop(){
  if(millis()-s100_time>100){
    gachValue = map(analogRead(gach_PIN),215,700,100,0);   
    if (gachValue < 0){
      gachValue=0;
    }
    if (gachValue>100){
      gachValue = 100;
    }
    //gachValue = 0;
    potValue = map(analogRead(pot_PIN),0,1023,100,0);
    if (potValue<10){
      potValue = 0;
    }
    
    sendToMotor();
    s100_time = millis();
    
  }
  
  //potValue = analogRead(pot_PIN);
}
void s250_loop(){
  if(millis() - s250_time > 250){
    if (millis() - interrupt_cad > 250){
      cadence = 0;
      dacVoltage = 0;
    }
    printStreamln(String(cadence) + "," + String(vitesse));
    printStreamln(String(gachValue)+ "," + String(analogRead(gach_PIN)));
    printStreamln(String(potValue)+ "," + String(analogRead(pot_PIN)));
    printStreamln(String(dacVoltage)+ "," + String(smoothedOut.get()));
    s250_time = millis();
  }  
}

void s1000_loop(){
  if(millis()-s1000_time > 1000){
    if(millis() - interrupt_vit >1000){
      vitesse = 0;
    }
    //Serial.println("s1000loop");
    s1000_time  =millis();
  }
}
void loop() {
  // put your main code here, to run repeatedly:
//  leds[0] = 0xf00000;
//  FastLED.show();
//  delay(500);
//  // Now turn the LED off, then pause
//  leds[0] = 0x00f000;
//  FastLED.show();
//  delay(500);
  s100_loop();
  s250_loop();
  s1000_loop();
}
