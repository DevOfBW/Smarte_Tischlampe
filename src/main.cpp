#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h" //only needed with flash breakpoints
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include <paj7620.h>


// Variablen:
#define LED_PIN_IndLi    6    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndLi 15    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite


// Funktionen:
int RGB_Licht_Funktion(int, int, int, int, int, int); //(Pixel,R,G,B,Helligkeit,Modus)
int Signalgeber(int, int); //(An, Modus)


// Objekte:
Adafruit_NeoPixel strip_IndLi(LED_COUNT_IndLi, LED_PIN_IndLi, NEO_GRB + NEO_KHZ800);    // NeoPixel pixel object:
// Argument 1 = Number of pixels in NeoPixel pixel
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
// NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
// NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
// NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)



// Setupmethode, diese Methode beeinhaltet alle Grundeinstellungen z.B. ob ein Kanal ein Eingang oder Ausgang ist. 
// Diese Mehtode wird einmalig zum Programmstart ausgeführt
void setup() 
{
  //Allgemein:
  debug_init(); //AUFRUF IST NOTWENDIG UM DEBUGGER ZU STARTEN
  //Serial.begin(9600); //Iinitialisierung von Serieller Verbindung um Ergebnisse anzuzeigen auf Konsole

  //Neopixel
  strip_IndLi.begin(); 
  strip_IndLi.show(); //Initialize all pixels from indirect light strip left to OFF

  //Signalgeber (Summer)
  pinMode(4, OUTPUT);

  //Gestensensor
   int error= paj7620Init();
  if(error)
  {
    Serial.print("Initilisierung Gestenseuerung fehlgeschlagen:");
    Serial.println(error);
  }else{
    Serial.println("Gestensensor ist ready");
  }
 
}


// Main-Code-Schleife, diese Methode wird ständig wiederholt
void loop() 
{
  //RGB_Licht_Funktion(0, 0, 0, 0, 255, 4);
  //Signalgeber(0,0);

   uint8_t gesture;//unsigned 8-bit integer
  int error = paj7620ReadReg(0x43,1,&gesture);

  if(!error)
  {
    switch(gesture)
    {
      case GES_RIGHT_FLAG:
        Serial.println("Right gesture");
        break;

      case GES_LEFT_FLAG:
        Serial.println("Left gesture");
        break;

      case GES_UP_FLAG:
        Serial.println("Up gesture");
        break;

      case GES_DOWN_FLAG:
        Serial.println("Down gesture");
        break;

      case GES_FORWARD_FLAG:
        Serial.println("Forward gesture");
        break;

      case GES_BACKWARD_FLAG:
        Serial.println("Backward gesture");
        break;

      case GES_CLOCKWISE_FLAG:
        Serial.println("Clockwise gesture");
        break;

      case GES_COUNT_CLOCKWISE_FLAG:
        Serial.println("Count Clockwise gesture");
        break;

      default:
        break;
    }
  }else{
    Serial.println("Gestensensor hat ein Problem: ");
    Serial.println(error);
  }

}

int Signalgeber(int an, int modi)
{
  tone(4,264); // (pin, frequency, duration)
  delay(1000);
  tone(4,547);
  delay(1000);
  noTone(4);
  return 1;
}

int RGB_Licht_Funktion(int pixelnummer, int rot, int gruen, int blau, int helligkeit, int modi)
{
  if(modi==1)  //Modi 1 = Arbeitslicht (ca.6000K) an + indirekte Beleuchtung in gleicher Farbe
  {
      rot=255;
      gruen=255;
      blau=255;
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndLi.show();
  }
  else if (modi==2)  //Modi 2 = Entspannungslicht (ca.4000K) an + indirekte Beleuchtung in gleicher Farbe
  {
      rot=241;
      gruen=142;
      blau=28;
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndLi.show();
  }
  else if (modi==3)  //Modi 3 = Farbenwechsel, Hauptleuchte und indirekte Beleuchtung wechselt die Farbe langsam
  {
    

  }
  else if (modi==4)  //Modi 4 = Partylicht, alle RGB wechseln die Farben schnell
  {
    for(int x1=0 ; x1<=14 ; x1++)
    {
      rot=255;
      gruen=0;
      blau=0;
      strip_IndLi.setPixelColor(x1, rot, gruen, blau);
      strip_IndLi.show(); 
      delay(50);
    }
    for(int x2=0 ; x2<=14 ; x2++)
    {
      rot=0;
      gruen=255;
      blau=0;
      strip_IndLi.setPixelColor(x2, rot, gruen, blau);
      strip_IndLi.show(); 
      delay(50);
    }
    for(int x3=0 ; x3<=14 ; x3++)
    {
      rot=0;
      gruen=0;
      blau=255;
      strip_IndLi.setPixelColor(x3, rot, gruen, blau);
      strip_IndLi.show(); 
      delay(50);
    }  
  }
  else if (modi==5)  //Modi 3 = Neopixel einzeln ansteuern
  {
    strip_IndLi.setPixelColor(pixelnummer, rot, gruen, blau);
    strip_IndLi.show(); 
  }
  return 1;
}