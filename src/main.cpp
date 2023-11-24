#include <Arduino.h>
//#include "avr8-stub.h"
//#include "app_api.h" //only needed with flash breakpoints
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include <avr/power.h>

#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include "RevEng_PAJ7620.h"

#include "TimeLib.h"
#include "DCF77.h"




// Variablen:
#define LED_PIN_IndLi    6    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndLi 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define LED_PIN_IndRe    5    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndRe 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define main_light_width 5
#define main_light_high 5
#define main_light_pin 3

#define DCF_PIN 2           // Connection pin to DCF 77 device
#define DCF_INTERRUPT 0    // Interrupt number associated with pin
#define PIN_LED 13
#define PIN_schalter 7

int helligkeit; //Wird benötigt für die LDR-Messung
char hmi_input [7]={}; //Es werden 7 char benötigt, da wir 7 Datensätze pro Button übertragen, um diesen zu identifizieren
bool flanke_Licht_ein = false;
int modus=0;
int rot=0;
int gruen=0;
int blau=0;
int leuchtstaerke=0;
bool hauptleuchte_an=false;
bool indirektebeleuchtung_an=false;
int durchlaufzaehler_party_farbwechsel=0;
volatile int flankenzaehler_ein_aus=0;


// Funktionen:
int RGB_Licht_Funktion(int, int, int, int, int, int, bool, bool); //(Pixel,R,G,B,Helligkeit,Modus,Hauptleuchte_an,Indirektebeleuchtung_an)
int Signalgeber(int, int); //(An, Modus)
int Gestensensor(); //Gestensensor
void digitalClockDisplay();  // DCF77
void printDigits(int);  // DCF77
int LDR_Messung(); //LDR Messung zwischen 0 und 1023
void Serielle_Textausgabe(String, String); //Textausgabe zum HMI
void HMI_Input_loeschen(char*);


// Objekte:
Adafruit_NeoPixel strip_IndLi(LED_COUNT_IndLi, LED_PIN_IndLi, NEO_GRB + NEO_KHZ800);    // NeoPixel pixel object:
Adafruit_NeoPixel strip_IndRe(LED_COUNT_IndRe, LED_PIN_IndRe, NEO_GRB + NEO_KHZ800);    // NeoPixel pixel object:
// Argument 1 = Number of pixels in NeoPixel pixel
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
// NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
// NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
// NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoMatrix main_light(main_light_width, main_light_high,main_light_pin, NEO_MATRIX_TOP+NEO_MATRIX_LEFT+NEO_MATRIX_ROWS ,NEO_GRB+NEO_KHZ800);

//Gestensensorobjekt
RevEng_PAJ7620 sensor = RevEng_PAJ7620();

//DCF77
//time_t time;
DCF77 DCF = DCF77(DCF_PIN, DCF_INTERRUPT);
// wurde ein gültiges Signal gefunden
bool g_bDCFTimeFound = false;



// Setupmethode, diese Methode beeinhaltet alle Grundeinstellungen z.B. ob ein Kanal ein Eingang oder Ausgang ist. 
// Diese Mehtode wird einmalig zum Programmstart ausgeführt
void setup() 
{

  //debug_init(); //AUFRUF IST NOTWENDIG UM DEBUGGER ZU STARTEN
  Serial.begin(9600); //Iinitialisierung von Serieller Verbindung um Ergebnisse anzuzeigen auf Konsole

  //Neopixel
  strip_IndLi.begin(); 
  strip_IndLi.show(); //Initialize all pixels from indirect light strip left to OFF
  strip_IndRe.begin(); 
  strip_IndRe.show(); //Initialize all pixels from indirect light strip left to OFF
  main_light.begin();
  main_light.show(); //Initialize all pixels from indirect light strip left to OFF

  //Signalgeber (Summer)
  pinMode(4, OUTPUT);

  //Gestensensor
  /*Serial.println("PAJ7620 sensor demo: Recognizing all 9 gestures.");

 if( !sensor.begin() )           // return value of 0 == success
  {
    Serial.print("PAJ7620 I2C error - halting");
  } 

  Serial.println("PAJ7620 init: OK");
  Serial.println("Please input your gestures:"); */

  //DCF77  
 /* pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_schalter, OUTPUT);
  digitalWrite(PIN_schalter, LOW);
  Serial.begin(9600); 
  DCF.Start();
  Serial.println("Warten auf DCF77-Zeit... ");
  Serial.println("Dies dauert mindestens 2 Minuten, in der Regel eher länger.");
  delay(2000); */
}


// Main-Code-Schleife, diese Methode wird ständig wiederholt
void loop() 
{
  //Signalgeber(0,0);
  //Gestensensor();

  Serial.println(Serial.available());
  //HMI
  if(Serial.available() > 0) //Prüfe ob Serielle Schnittstelle erreichbar
  {
    for(int i=0;i<6;i++) //Eingehende Nummer von Inputs einlesen (xx yy zz dd aa uu ii) yy=Seite, zz=Button 1,2,3
    {
      hmi_input[i]=hmi_input[i+1];
      Serial.println(hmi_input[i]);
    }
    hmi_input[6]=Serial.read();
  }

  
  if(hmi_input[1]==1 && (hmi_input[2]==3 || hmi_input[2]==4 || hmi_input[2]==5 || hmi_input[2]==6 || hmi_input[2]==7)) //hmi_input[1]=> Seite 2, hmi_input[2] => Buttons Modies
  {
    switch (hmi_input[2])
    {
    case 3:
      modus=1; //Arbeitslicht (Lernen)
      rot=255;
      gruen=255;
      blau=255;
      leuchtstaerke=255;
      Serielle_Textausgabe("texbox_1_lk.txt=","Lernen"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 4:
      modus=2; //Entspannungslicht (Relax)
      rot=241;
      gruen=142;
      blau=28;
      leuchtstaerke=255;
      Serielle_Textausgabe("texbox_1_lk.txt=","Relax"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 5:
      modus=4; //Party
      leuchtstaerke=255;
      Serielle_Textausgabe("texbox_1_lk.txt=","Party"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 6:
      modus=3; //Farben mix
      Serielle_Textausgabe("texbox_1_lk.txt=","Farbenmix"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 7:
      modus=6; //Lichtabhängige Lichtansteuerung (Automatik)
      Serielle_Textausgabe("texbox_1_lk.txt=","Automatik"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    
    default:
      break;
    }
  }



  if(hmi_input[1]==1 && hmi_input[2]==0x0F && indirektebeleuchtung_an==false) //Auswertung welche Leuchten angesteuert werden sollen
  {
    indirektebeleuchtung_an=true;
    HMI_Input_loeschen(hmi_input);
  }
  else if(hmi_input[1]==1 && hmi_input[2]==0x0F && indirektebeleuchtung_an==true)
  {
    indirektebeleuchtung_an=false;
    HMI_Input_loeschen(hmi_input);
  }
  if(hmi_input[1]==1 && hmi_input[2]==0x0E && hauptleuchte_an==false)
  {
    hauptleuchte_an=true;
    HMI_Input_loeschen(hmi_input);
  }
  else if(hmi_input[1]==1 && hmi_input[2]==0x0E && hauptleuchte_an==true)
  {
    hauptleuchte_an=false;
    HMI_Input_loeschen(hmi_input);
  }

  if(hmi_input[1]==0 && hmi_input[2]==5 && modus!=0)  //Ein/Aus der ausgewählten Leuchten
  {
    flankenzaehler_ein_aus++;
    if(flankenzaehler_ein_aus==1)
    {
      flanke_Licht_ein=true;
      Serielle_Textausgabe("texbox_1.txt=","Licht Ein"); //Ausgabetext in Textbox 1 auf Seite 1
      if(modus!=4 || modus !=6){
        RGB_Licht_Funktion(0, rot, gruen, blau, leuchtstaerke, modus, hauptleuchte_an, indirektebeleuchtung_an);
      }  
    } 
    else if(flankenzaehler_ein_aus==2)
    {
      flanke_Licht_ein=false;
      flankenzaehler_ein_aus=0;
      Serielle_Textausgabe("texbox_1.txt=","Licht Aus"); //Ausgabetext in Textbox 1 auf Seite 1 
      RGB_Licht_Funktion(0, 0, 0, 0, 0, modus, hauptleuchte_an, indirektebeleuchtung_an);
    }
     HMI_Input_loeschen(hmi_input);
  }
   

  if(modus==4 && flanke_Licht_ein==true) //Farbwechsel für Partymodus
  {
    if(durchlaufzaehler_party_farbwechsel<=100)
    {
      rot=255;
      gruen=0;
      blau=0;
    }
    else if(durchlaufzaehler_party_farbwechsel<=200)
    {
      rot=0;
      gruen=255;
      blau=0;
    }
    else if(durchlaufzaehler_party_farbwechsel<=300)
    {
      rot=0;
      gruen=0;
      blau=255;
    }
    else if(durchlaufzaehler_party_farbwechsel==301)
    {
      durchlaufzaehler_party_farbwechsel=0;
    }
    durchlaufzaehler_party_farbwechsel++;
    RGB_Licht_Funktion(0, rot, gruen, blau, 255, modus, hauptleuchte_an, indirektebeleuchtung_an);
  }


  if(modus==6 && flanke_Licht_ein==true) //Automatikmodus
  {
    int aktueller_LDR_wert=LDR_Messung();
    int setze_helligkeit;

      if(aktueller_LDR_wert<=300)
      {
        setze_helligkeit=255;
      }
      else if (aktueller_LDR_wert<=500)
      {
        setze_helligkeit=200;
      }
      else if (aktueller_LDR_wert<=700)
      {
        setze_helligkeit=150;
      }
      else if (aktueller_LDR_wert<=850)
      {
        setze_helligkeit=50;
      }
      else if (aktueller_LDR_wert<=1000)
      {
        setze_helligkeit=0;
      }
    rot=241;
    gruen=142;
    blau=28;
    RGB_Licht_Funktion(0, rot, gruen, blau, setze_helligkeit, modus, hauptleuchte_an, indirektebeleuchtung_an);
  }
 
  
// das Signal wird nur aller 5 Sekunden abgefragt
 /* delay(950);
  digitalWrite(PIN_LED, HIGH);
  delay(50);
  digitalWrite(PIN_LED, LOW);
  time_t DCFtime = DCF.getTime(); // Check if new DCF77 time is available
  if (DCFtime!=0)
  {
    Serial.println("Time is updated");
    setTime(DCFtime);
    g_bDCFTimeFound = true;
  }
  
  // die Uhrzeit wurde gesetzt, also LED nach kurzer Zeit ein
  if (g_bDCFTimeFound)
  {
    delay(50);
    digitalWrite(PIN_LED, HIGH);
  }
  digitalClockDisplay();*/
}




void HMI_Input_loeschen(char* HMI_Input_array)
{
  for(int i=0; i<7;i++) //Inputdatenarray löschen
      {
        HMI_Input_array[i]=0;
      }
}

void Serielle_Textausgabe(String textbox, String text)
{
  String cmd;
  cmd +="\"";
  for(int i=0;i<=2;i++){
      Serial.print(textbox + cmd + text + cmd);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
  }
}

int LDR_Messung()
{
  helligkeit = analogRead(0);  
  return helligkeit;
}

void DCF77()
{
   
}

void digitalClockDisplay()
{
  // digital clock display of the time
  printDigits(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println();
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

int Gestensensor()
{
Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
  gesture = sensor.readGesture();   // Read back current gesture (if any) of type Gesture

  switch (gesture)
  {
    case GES_FORWARD:
      {
        Serial.println("GES_FORWARD");
        break;
      }

    case GES_BACKWARD:
      {
        Serial.println("GES_BACKWARD");
        break;
      }

    case GES_LEFT:
      {
        Serial.println("GES_LEFT");
        break;
      }

    case GES_RIGHT:
      {
        Serial.println("GES_RIGHT");
        break;
      }

    case GES_UP:
      {
        Serial.println("GES_UP");
        break;
      }

    case GES_DOWN:
      {
        Serial.println("GES_DOWN");
        break;
      }

    case GES_CLOCKWISE:
      {
        Serial.println("GES_CLOCKWISE");
        break;
      }

    case GES_ANTICLOCKWISE:
      {
        Serial.println("GES_ANTICLOCKWISE");
        break;
      }

    case GES_WAVE:
      {
        Serial.println("GES_WAVE");
        break;
      }

    case GES_NONE:
      {
        break;
      }
  }

  delay(100);
  return 1;
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

int RGB_Licht_Funktion(int pixelnummer, int rot, int gruen, int blau, int helligkeit, int modi, bool hauptleuchte, bool indirekt)
{
  if(modi==1)  //Modi 1 = Arbeitslicht (ca.6000K) an + indirekte Beleuchtung in gleicher Farbe
  {
    strip_IndLi.setBrightness(helligkeit);
    strip_IndRe.setBrightness(helligkeit);
    main_light.setBrightness(helligkeit);
   
    if(indirekt==true && hauptleuchte==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
      strip_IndLi.show();
      strip_IndRe.show();
    }
    if(indirekt==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      strip_IndLi.show();
      strip_IndRe.show();
    }
    else if(hauptleuchte==true)
    {
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
    }
  }
  else if (modi==2)  //Modi 2 = Entspannungslicht (ca.4000K) an + indirekte Beleuchtung in gleicher Farbe
  {
    strip_IndLi.setBrightness(helligkeit);
    strip_IndRe.setBrightness(helligkeit);
    main_light.setBrightness(helligkeit);

    if(indirekt==true && hauptleuchte==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
      strip_IndLi.show();
      strip_IndRe.show();
    }
    if(indirekt==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      strip_IndLi.show();
      strip_IndRe.show();
    }
    else if(hauptleuchte==true)
    {
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
    }
  }
  else if (modi==3)  //Modi 3 = Farbe selber mixen
  {
    

  }
  else if (modi==4)  //Modi 4 = Partylicht, alle RGB wechseln die Farben schnell
  {
    strip_IndLi.setBrightness(helligkeit);
    strip_IndRe.setBrightness(helligkeit);
    main_light.setBrightness(helligkeit);
    if(indirekt==true && hauptleuchte==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
      strip_IndLi.show();
      strip_IndRe.show();
    }
    if(indirekt==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      strip_IndLi.show();
      strip_IndRe.show();
    }
    else if(hauptleuchte==true)
    {
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
    }
  }
  else if (modi==5)  //Modi 3 = Neopixel einzeln ansteuern
  {
    strip_IndLi.setPixelColor(pixelnummer, rot, gruen, blau);
    strip_IndLi.show(); 
  }
  else if (modi==6) //Modi 6 = Helligkeitsabhänging die LED-Lichtstärke steuern
  {
    strip_IndLi.setBrightness(helligkeit);
    strip_IndRe.setBrightness(helligkeit);
    main_light.setBrightness(helligkeit);

    if(indirekt==true && hauptleuchte==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
      strip_IndLi.show();
      strip_IndRe.show();
    }
    if(indirekt==true)
    {
      strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
      strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
      strip_IndLi.show();
      strip_IndRe.show();
    }
    else if(hauptleuchte==true)
    {
      main_light.fillScreen(main_light.Color(rot, gruen, blau));
      main_light.show();
    }
  } 
  return 1;
}