// 19.12.2023 RAM: 39,6%  Flash: 42,1%
// 20.12.2023 RAM: 33,3%  Flash: 40,7%
// 23.12.2023 RAM: 36,8%  Flash: 39,3% (Uhrzeit implementiert)

#include <Arduino.h>
#include <avr/power.h>

#include "Nextion.h"
#include "Adafruit_NeoPixel.h"  //Indirekte Beleuchtung
#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include "RevEng_PAJ7620.h"  //Gestensensor
#include "RTClib.h"  //Realtime-Clock

// Variablen:
#define LED_PIN_IndLi    6    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndLi 29    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define LED_PIN_IndRe    5    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndRe 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define main_light_count 25
#define main_light_pin 3
#define PIN_LED 13
#define PIN_schalter 7
#define PIN_summer 4 //Pin an dem der Summer angeschlossen ist
#define PIN_SQW 2
//const byte PIN_SQW = 2; //Interruptpin RTC -> SQW pin is used to monitor the SQW 1Hz output from the DS3231
bool flanke_rtc_sqw; //A variable to store when a falling 1Hz clock edge has occured on the SQW pin of the DS3231
uint8_t helligkeit; //Wird benötigt für die LDR-Messung
bool flanke_Licht_ein = false;
uint8_t modNeu=0;
uint8_t parRed[12]={128, 192, 239, 255, 239, 192, 128, 64, 17, 0, 17, 64};
uint8_t parBlue[13]={213, 159, 97, 43, 8, 1, 23, 69, 128, 187, 233, 255, 248};
uint8_t parGreen[14]={2, 5, 32, 78, 134, 189, 232, 254, 251, 224, 178, 122, 67, 24};
uint8_t parBright[11]={128, 169, 196, 202, 185, 149, 107, 71, 54, 60, 87};
int count=0;
int pause=0;
uint8_t page=0; //0 Main-Seite

//Speicher für den Mixmodus
uint8_t indRed,indGreen,indBlue,indBright;
uint8_t mainRed,mainGreen,mainBlue,mainBright;
uint32_t memory;
uint8_t modus=0;  //1 Lernen, 2 Relax, 3 Mix, 4 Party, 6 Auto
uint8_t leuchtstaerke=0;
bool hauptleuchte_an=false;
bool indirektebeleuchtung_an=false;
bool radioH=false;
bool radioN=false;
int durchlaufzaehler_party_farbwechsel=0;
volatile uint8_t flankenzaehler_ein_aus=0;
uint8_t activeLamp=0; //0 beide aus; 1 Haupt; 2 Neben
uint8_t red,green,blue,bright;

//TODO: Können die Bool-Werte nicht einfacher in zwei Integern gespeichert werden? Wäre auch leichter zu verstehen.
char hmi_input [7]={};    //Es werden 4 char benötigt, da wir 4 Datensätze pro Button übertragen, um diesen zu identifizieren
bool montag_alarm1_memory=true;
bool montag_alarm2_memory=true;
bool dienstag_alarm1_memory=true;
bool dienstag_alarm2_memory=true;
bool mittwoch_alarm1_memory=true;
bool mittwoch_alarm2_memory=true;
bool donnerstag_alarm1_memory=true;
bool donnerstag_alarm2_memory=true;
bool freitag_alarm1_memory=true;
bool freitag_alarm2_memory=true;
bool samstag_alarm1_memory=true;
bool samstag_alarm2_memory=true;
bool sonntag_alarm1_memory=true;
bool sonntag_alarm2_memory=true;
bool alarm1_ein_memory=false;
bool alarm2_ein_memory=false;
int8_t alarm1_minute_memory;
int8_t alarm1_stunde_memory;
int8_t alarm2_minute_memory;
int8_t alarm2_stunde_memory;
int help=0;


// Funktionen:
void Signalgeber(bool); //(An/Aus)
uint8_t Gestensensor(); //Gestensensor
uint8_t LDR_Messung(); //LDR Messung zwischen 0 und 1023
void Serielle_Textausgabe(const char*, const char*); //Textausgabe zum HMI
void ISR_RTC ();  //Interrupt Service routine von RTC modul ausgelöst durch SQW
void displayTime (bool); //Ausgabe der aktuellen Zeit
void HMI_Input_loeschen(char*);
boolean setModusActive(int);
void partymodus();
void refreshColours();
void activateIndBel(uint8_t, uint8_t, uint8_t, uint8_t);
void activateHauptBel(uint8_t, uint8_t, uint8_t, uint8_t);
void sendValue(const char*, uint8_t);

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
Adafruit_NeoPixel main_light(main_light_count,main_light_pin, NEO_GRB + NEO_KHZ800);

//Gestensensorobjekt
RevEng_PAJ7620 sensor = RevEng_PAJ7620();

// TODO: Alle Displayelemente und die Funktionen extrahieren

//Displayelemente
NexSlider p01 = NexSlider(6, 1, "p01"); //Slider initialisieren rot; Touch-Release Event muss noch konfiguriert werden
NexSlider p03 = NexSlider(6, 4, "p03"); //Slider gruen
NexSlider p05 = NexSlider(6, 6, "p05"); //Slider blau
NexSlider p07 = NexSlider(6, 8, "p07");  //Slider helligkeit

#pragma region DisplayFunctions

void p01PopCallback(){
  p01.getValue(&memory);
  red=memory;
  refreshColours();
}

void p03PopCallback(){
  p03.getValue(&memory);
  green=memory;
  refreshColours();
}

void p05PopCallback(){
  p05.getValue(&memory);
  blue=memory;
  refreshColours();
}

void p07PopCallback(){
  p07.getValue(&memory);
  bright=memory;
  refreshColours();
}

//Die Farbwerte für den aktuellen Modus speichern
//Speichert die Werte für die aktiven Lampen
void bt_save_lsPopCallback(){
  if(activeLamp==2){
    indRed=red;
    indBlue=blue;
    indGreen=green;
    indBright=bright;
  }else if(activeLamp==1){
    mainRed=red;
    mainBlue=blue;
    mainGreen=green;
    mainBright=bright;
  }
}

void b_switch_lsPopCallback(){
  char text[6];
  strcpy(text,"      ");
  activeLamp++;
  switch(activeLamp){
    case 1: 
        hauptleuchte_an=true;
        indirektebeleuchtung_an=false;
        strcpy(text,"Haupt");
        red=mainRed;
        blue=mainBlue;
        green=mainGreen;
        bright=mainBright;
        break;
    case 2: 
        hauptleuchte_an=false;
        indirektebeleuchtung_an=true;
        strcpy(text,"Neben");
        red=indRed;
        blue=indBlue;
        green=indGreen;
        bright=indBright;
        break;
    default: 
        hauptleuchte_an=false;
        indirektebeleuchtung_an=false;
        strcpy(text,"Aus");
        activeLamp=0;
        break;
  }
  Serielle_Textausgabe("p10.txt=",text);
  sendValue("p09",0);
  refreshColours();
  //delay(10);
  sendValue("p08",bright);
  //delay(10);
  sendValue("p02",red);
  //delay(10);
  sendValue("p06",blue);
  //delay(10);
  sendValue("p04",green);
  //delay(10);
  sendValue("p07",bright);
  //delay(10);
  sendValue("p01",red);
  //delay(10);
  sendValue("p05",blue);
  //delay(10);
  sendValue("p03",green);
  refreshColours();
}

//Konfigurationsbutton
void b_mixco_lkPopCallback(){
  setModusActive(3);
}

void r_hauptle_lkPopCallback(){
  radioH=!radioH;
  hauptleuchte_an=radioH;
  refreshColours();
}

void r_indirektb_lkPopCallback(){
  radioN=!radioN;
  indirektebeleuchtung_an=radioN;
  refreshColours();
}
#pragma endregion
//Wecker + Uhrzeit
RTC_DS3231 rtc;
char wochentage[7][3] = {"So","Mo", "Di", "Mi", "Do", "Fr", "Sa"};
char monate_des_jahres[12][12] = {"Januar", "Februar", "Maerz", "April", "Mai", "Juni","Juli", "August", "September", "Oktober", "November", "Dezember"}; 


// Setupmethode, diese Methode beeinhaltet alle Grundeinstellungen z.B. ob ein Kanal ein Eingang oder Ausgang ist. 
// Diese Mehtode wird einmalig zum Programmstart ausgeführt.
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
  pinMode(PIN_summer, OUTPUT); 

  //Gestensensor
  
  Serial.println("PAJ7620 sensor demo: Recognizing all 9 gestures.");

  if( !sensor.begin() )           // return value of 0 == success
  {
    Serial.println("PAJ7620 I2C error - halting");
  }else{
    Serial.println("PAJ7620 init: OK");
    Serial.println("Please input your gestures:");
  }
  
  //RTC
  if (! rtc.begin()) {
    //TODO: Ausgabe der Fehlermeldung auf Touchdisplay
    Serial.println("RTC nicht gefunden");
    Serial.flush();
    abort();
  }else{
    Serial.println("RTC initialisiert");
  }

   rtc.disable32K();

  if (rtc.lostPower()) {
    //TODO: Implementieren auf Touchdisplay evtl. sichtbar
     Serial.println("RTC lost power, let's set the time!");
  }
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);   //Configure SQW pin on the DS3231 to output a 1Hz squarewave to Arduino pin 2 (SQWinput) for timing
  pinMode(PIN_SQW, INPUT); //Configure the SQWinput pin as an INPUT to monitor the SQW pin of the DS3231.
  digitalWrite (PIN_SQW, HIGH); //Enable the internal pull-up resistor, since the SQW pin on the DS3231 is an Open Drain output.
  attachInterrupt(digitalPinToInterrupt(PIN_SQW), ISR_RTC, FALLING); //Configure SQWinput (pin 2 of the Arduino) for use by the Interrupt Service Routine (Isr)
  flanke_rtc_sqw = true; //Initialize EDGE equal to 1. The Interrupt Service Routine will make EDGE equal to zero when triggered by a falling clock edge on SQW

  //Alarm (Wecker)
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);  
  rtc.writeSqwPinMode(DS3231_OFF);
  

}


// Main-Code-Schleife, diese Methode wird ständig wiederholt
void loop() 
{
  DateTime now = rtc.now();

  //HMI input bitstream lesen
  if(Serial.available() > 0) //Prüfe ob Serielle Schnittstelle erreichbar
  {
    for(int i=0;i<6;i++) //Eingehende Nummer von Inputs einlesen (xx yy zz dd aa uu ii) yy=Seite, zz=Button 1,2,3
    {
      hmi_input[i]=hmi_input[i+1];
    }
    hmi_input[6]=Serial.read();
  }


//Äußere switch seite
//switch in Case von äußerem switch button
//hmi_input[1]=> Seite 2, hmi_input[2] => Button
switch (hmi_input[1])
{
  case 0x00: //Mainpage
      if(hmi_input[2]==1||hmi_input[2]==2||hmi_input[2]==3){
        page=hmi_input[2];
      }
      HMI_Input_loeschen(hmi_input);
  break;



  case 0x01: //Licht-Konfig-page
      switch(hmi_input[2]){
        case 0x02://Home-Button
          page=0;
          break;
        case 0x0B://Lernen-Button
          setModusActive(1);
          break;
        case 0x0D://Relax-Button
          setModusActive(2);
          break;
        case 0x0C://Automatik-Button
          setModusActive(6);
          break;
        case 0x0F://Farbenmix-Button
          setModusActive(3);
          break;
        case 0x0E://Party-Button
          setModusActive(4);
          break;
        case 0x10://Licht-Konfig
          b_mixco_lkPopCallback();
          break;
        case 0x09://Hauptlicht an?
          r_hauptle_lkPopCallback();
          break;
        case 0x0A://Nebenlicht an?
          r_indirektb_lkPopCallback();
          break;
      }
      HMI_Input_loeschen(hmi_input);
  break;



  case 2: //Wecker-page
  #pragma region Wecker
      char alarm1_stunde[4];
      char alarm1_minute[4];
      char alarm2_stunde[4];
      char alarm2_minute[4];
      
        switch (hmi_input[2])
        {
          case 0x02:  //Home-Button
            page=0;
            break;
          case 0x14: //Alarm 1Stunde verringern(1, 0, 23,...)
            alarm1_stunde_memory--;
            if(alarm1_stunde_memory<0){
            alarm1_stunde_memory=23;
            }
            sprintf(alarm1_stunde, "%02d", alarm1_stunde_memory);
            Serielle_Textausgabe("a10.txt=",alarm1_stunde);
            break;
          case 0x12: //Alarm 1 Stunde erhöhen (1, 2, 3,...)
            alarm1_stunde_memory++;
            if(alarm1_stunde_memory>23){
            alarm1_stunde_memory=0;
            }
            sprintf(alarm1_stunde, "%02d", alarm1_stunde_memory);
            Serielle_Textausgabe("a10.txt=",alarm1_stunde);
            break; 

          case 0x17: //Alarm 1 Minute verringern(1, 59, 58,...)
            alarm1_minute_memory--;
            if(alarm1_minute_memory<1){
            alarm1_minute_memory=59;
            }
            sprintf(alarm1_minute, "%02d", alarm1_minute_memory);
            Serielle_Textausgabe("a12.txt=",alarm1_minute);
            break;
          case 0x15: //Alarm 1 Minute erhöhen (1, 2, 3,...)
            alarm1_minute_memory++;
            if(alarm1_minute_memory>59){
            alarm1_minute_memory=0;
            }
            sprintf(alarm1_minute, "%02d", alarm1_minute_memory);
            Serielle_Textausgabe("a12.txt=",alarm1_minute);
            break;

          case 0x04: //Montag (Alarm1)
              if(montag_alarm1_memory==true)
              {
                montag_alarm1_memory=false;
              } else if (montag_alarm1_memory==false)
              {
                montag_alarm1_memory=true;
              } 
            break;
          case 0x05: //Dienstag (Alarm1)
              if(dienstag_alarm1_memory==true)
              {
                dienstag_alarm1_memory=false;
              } else if (dienstag_alarm1_memory==false)
              {
                dienstag_alarm1_memory=true;
              }     
            break;
          case 0x07: //Mittwoch (Alarm1)
              if(mittwoch_alarm1_memory==true)
              {
                mittwoch_alarm1_memory=false;
              } else if (mittwoch_alarm1_memory==false)
              {
                mittwoch_alarm1_memory=true;
              } 
            break;
          case 0x06: //Donnerstag (Alarm1)
              if(donnerstag_alarm1_memory==true)
              {
                donnerstag_alarm1_memory=false;
              } else if (donnerstag_alarm1_memory==false)
              {
                donnerstag_alarm1_memory=true;
              } 
            break;
          case 0x09: //Freitag (Alarm1)
              if(freitag_alarm1_memory==true)
              {
                freitag_alarm1_memory=false;
                Serial.println("FR aus");
              } else if (freitag_alarm1_memory==false)
              {
                freitag_alarm1_memory=true;
                Serial.println("FR ein");
              } 
            break;
          case 0x08:  //Samstag (Alarm1)
              if(samstag_alarm1_memory==true)
              {
                samstag_alarm1_memory=false;
              } else if (samstag_alarm1_memory==false)
              {
                samstag_alarm1_memory=true;
              } 
            break;
          case 0x0A: //Sonntag (Alarm1)
              if(sonntag_alarm1_memory==true)
              {
                sonntag_alarm1_memory=false;
              } else if (sonntag_alarm1_memory==false)
              {
                sonntag_alarm1_memory=true;
              } 
            break;

          case 0x1C: //Alarm 2Stunde verringern(1, 0, 23,...)
            alarm2_stunde_memory--;
            if(alarm2_stunde_memory<0){
            alarm2_stunde_memory=23;
            }
            sprintf(alarm2_stunde, "%02d", alarm2_stunde_memory);
            Serielle_Textausgabe("a17.txt=",alarm2_stunde);
            break;
          case 0x1E: //Alarm 2 Stunde erhöhen (1, 2, 3,...)
            alarm2_stunde_memory++;
            if(alarm2_stunde_memory>23){
            alarm2_stunde_memory=0;
            }
            sprintf(alarm2_stunde, "%02d", alarm2_stunde_memory);
            Serielle_Textausgabe("a17.txt=",alarm2_stunde);
            break; 

          case 0x19: //Alarm 2 Minute verringern(1, 59, 58,...)
            alarm2_minute_memory--;
            if(alarm2_minute_memory<1){
            alarm2_minute_memory=59;
            }
            sprintf(alarm2_minute, "%02d", alarm2_minute_memory);
            Serielle_Textausgabe("a19.txt=",alarm2_minute);
            break;
          case 0x1B: //Alarm 2 Minute erhöhen (1, 2, 3,...)
            alarm2_minute_memory++;
            if(alarm2_minute_memory>59){
            alarm2_minute_memory=0;
            }
            sprintf(alarm2_minute, "%02d", alarm2_minute_memory);
            Serielle_Textausgabe("a19.txt=",alarm2_minute);
            break;

          case 0x0B: //Montag (Alarm2)
              if(montag_alarm2_memory==true)
              {
                montag_alarm2_memory=false;
              } else if (montag_alarm2_memory==false)
              {
                montag_alarm2_memory=true;
              } 
            break;
          case 0x0C: //Dienstag (Alarm2)
              if(dienstag_alarm2_memory==true)
              {
                dienstag_alarm2_memory=false;
              } else if (dienstag_alarm2_memory==false)
              {
                dienstag_alarm2_memory=true;
              }     
            break;
          case 0x0E: //Mittwoch (Alarm2)
              if(mittwoch_alarm2_memory==true)
              {
                mittwoch_alarm2_memory=false;
              } else if (mittwoch_alarm2_memory==false)
              {
                mittwoch_alarm2_memory=true;
              } 
            break;
          case 0x0D: //Donnerstag (Alarm2)
              if(donnerstag_alarm2_memory==true)
              {
                donnerstag_alarm2_memory=false;
              } else if (donnerstag_alarm2_memory==false)
              {
                donnerstag_alarm2_memory=true;
              } 
            break;
          case 0x10: //Freitag (Alarm2)
              if(freitag_alarm2_memory==true)
              {
                freitag_alarm2_memory=false;
              } else if (freitag_alarm2_memory==false)
              {
                freitag_alarm2_memory=true;
              } 
            break;
          case 0x0F:  //Samstag (Alarm2)
              if(samstag_alarm2_memory==true)
              {
                samstag_alarm2_memory=false;
              } else if (samstag_alarm2_memory==false)
              {
                samstag_alarm2_memory=true;
              } 
            break;
          case 0x11: //Sonntag (Alarm2)
              if(sonntag_alarm2_memory==true)
              {
                sonntag_alarm2_memory=false;
              } else if (sonntag_alarm2_memory==false)
              {
                sonntag_alarm2_memory=true;
              } 
            break;

          case 0x2D: //Alarm 1 Ein/Aus
              if(alarm1_ein_memory==true)
              {
                rtc.clearAlarm(1);
                alarm1_ein_memory=false;
                Serial.println("Alarm_1_ausgeschaltet");
              } else if (alarm1_ein_memory==false)
              {
                alarm1_ein_memory=true;
                Serial.println("Alarm_1_eingeschalten");
              } 
            break;
          case 0x2E: //Alarm 2 Ein/Aus
              if(alarm2_ein_memory==true)
              {
                rtc.clearAlarm(1);
                alarm2_ein_memory=false;
                Serial.println("Alarm_2_ausgeschaltet");
              } else if (alarm2_ein_memory==false)
              {
                alarm2_ein_memory=true;
                Serial.println("Alarm_2_eingeschalten");
              } 
            break;
          case 0x03: //save
            break;

            default:
              break;
        }
        HMI_Input_loeschen(hmi_input); 
  break;
  #pragma endregion

  case 0x03: //Settings-page
      switch (hmi_input[2])
      {
        case 0x02: //Home-Button
          page=0;
          break;
      case 0x03:
        //displayTime(true);
        break;
      
      default:
        break;
      }
      HMI_Input_loeschen(hmi_input);
  break;



  case 0x04: //Gestensteuerung-page
      if(hmi_input[2]==0x03){
        page=0;
      }
      HMI_Input_loeschen(hmi_input);
  break;



  case 5: //Uhr-Konfig-page
  #pragma region UhrKonfig
      //DateTime now = rtc.now();
      static int8_t tag_memory=now.day();
      char tag[3];
      static int8_t monat_memory=now.month();
      char monat[3];
      static int16_t jahr_memory=now.year();
      char jahr[6];
      static int8_t stunde_memory=now.hour();
      char stunde[4];
      static int8_t minute_memory=now.minute();
      char minute[4];

      switch (hmi_input[2])
      {
        case 0x02: //Home-Button
          page=0;
          break;
      case 0x0C: //Tag verringern(1, 31, 30,...)
        tag_memory--;
        if(tag_memory<1){
        tag_memory=31;
        }
        sprintf(tag, "%02d", tag_memory);
        Serielle_Textausgabe("u05.txt=",tag);
        break;
      case 0x0A: //Tag erhöhen (1, 2, 3,...)
        tag_memory++;
        if(tag_memory>31){
        tag_memory=1;
        }
        sprintf(tag, "%02d", tag_memory);
        Serielle_Textausgabe("u05.txt=",tag);
        break;

      case 0x0F: //Monat verringern(1, 12, 1,...)
        monat_memory--;
        if(monat_memory<1){
        monat_memory=12;
        }
        sprintf(monat, "%02d", monat_memory);
        Serielle_Textausgabe("u08.txt=",monat);
        break;
      case 0x0D: //Monat erhöhen (1, 2, 3,...)
        monat_memory++;
        if(monat_memory>12){
        monat_memory=1;
        }
        sprintf(monat, "%02d", monat_memory);
        Serielle_Textausgabe("u08.txt=",monat);
        break;

      case 0x12: //Jahr verringern
        jahr_memory--;
        if(jahr_memory<1){
        jahr_memory=9999;
        }
        sprintf(jahr, "%04d", jahr_memory);
        Serielle_Textausgabe("u11.txt=",jahr);
        break;
      case 0x10: //Jahr erhöhen 
        jahr_memory++;
        if(jahr_memory>9999){
        jahr_memory=1;
        }
        sprintf(jahr, "%04d", jahr_memory);
        Serielle_Textausgabe("u11.txt=",jahr);
        break;

      case 0x15: //Stunde verringern(1, 0, 23,...)
        stunde_memory--;
        if(stunde_memory<0){
        stunde_memory=23;
        }
        sprintf(stunde, "%02d", stunde_memory);
        Serielle_Textausgabe("u14.txt=",stunde);
        break;
      case 0x13: //Stunde erhöhen (1, 2, 3,...)
        stunde_memory++;
        if(stunde_memory>23){
        stunde_memory=0;
        }
        sprintf(stunde, "%02d", stunde_memory);
        Serielle_Textausgabe("u14.txt=",stunde);
        break; 

      case 0x18: //Minute verringern(1, 59, 58,...)
        minute_memory--;
        if(minute_memory<1){
        minute_memory=59;
        }
        sprintf(minute, "%02d", minute_memory);
        Serielle_Textausgabe("u18.txt=",minute);
        break;
      case 0x16: //Minute erhöhen (1, 2, 3,...)
        minute_memory++;
        if(minute_memory>59){
        minute_memory=0;
        }
        sprintf(minute, "%02d", minute_memory);
        Serielle_Textausgabe("u18.txt=",minute);
        break;

      case 0x04: //Speichern
        rtc.adjust(DateTime(jahr_memory, monat_memory, tag_memory, stunde_memory, minute_memory, 0)); //(Jahr, Monat, Tag, Stunde, Minute, Sekunde)
        break;

      default:
        break;
      }
      HMI_Input_loeschen(hmi_input);
      break;
  #pragma endregion

  case 0x06: //Farbmix-page
      switch(hmi_input[2]){
        case 0x02:
          bt_save_lsPopCallback();
          break;
        case 0x0F:
          b_switch_lsPopCallback();
          break;
        case 0x01:
          p01PopCallback();
          break;
        case 0x04:
          p03PopCallback();
          break;
        case 0x06:
          p05PopCallback();
          break;
        case 0x08:
          p07PopCallback();
          break;
        case 0x10: //Home-Button
          page=0;
          break;
      }
      HMI_Input_loeschen(hmi_input);
  break;

  case 7:
    if(hmi_input[2]==3){
      page=0;
    }
  break;

  case 8:
    if(hmi_input[2]==1){
      page=0;
    }
  break;

default:
  break;
}


//Wecker (Alarm 1)
if((now.dayOfTheWeek()==1 && montag_alarm1_memory==true && alarm1_ein_memory==true) || 
    (now.dayOfTheWeek()==2 && dienstag_alarm1_memory==true && alarm1_ein_memory==true) || 
    (now.dayOfTheWeek()==3 && mittwoch_alarm1_memory==true && alarm1_ein_memory==true) ||
    (now.dayOfTheWeek()==4 && donnerstag_alarm1_memory==true && alarm1_ein_memory==true) ||
    (now.dayOfTheWeek()==5 && freitag_alarm1_memory==true && alarm1_ein_memory==true) ||
    (now.dayOfTheWeek()==6 && samstag_alarm1_memory==true && alarm1_ein_memory==true) ||
    (now.dayOfTheWeek()==0 && sonntag_alarm1_memory==true && alarm1_ein_memory==true))
    {
      rtc.setAlarm1(DateTime(now.year(), now.month(), now.day(), alarm1_stunde_memory, alarm1_minute_memory, 0), DS3231_A1_Day); //Alarm when day (day of week), hours,inutes and seconds match 
    }else{
      rtc.clearAlarm(1);  //Alarm ausschalten
      noTone(4);  //Signalton ausschalten
    }
//Wecker (Alarm 2)
if((now.dayOfTheWeek()==1 && montag_alarm2_memory==true && alarm2_ein_memory==true) || 
    (now.dayOfTheWeek()==2 && dienstag_alarm2_memory==true && alarm2_ein_memory==true) || 
    (now.dayOfTheWeek()==3 && mittwoch_alarm2_memory==true && alarm2_ein_memory==true) ||
    (now.dayOfTheWeek()==4 && donnerstag_alarm2_memory==true && alarm2_ein_memory==true) ||
    (now.dayOfTheWeek()==5 && freitag_alarm2_memory==true && alarm2_ein_memory==true) ||
    (now.dayOfTheWeek()==6 && samstag_alarm2_memory==true && alarm2_ein_memory==true) ||
    (now.dayOfTheWeek()==0 && sonntag_alarm2_memory==true && alarm2_ein_memory==true))
    {
      rtc.setAlarm2(DateTime(now.year(), now.month(), now.day(), alarm2_stunde_memory, alarm2_minute_memory, 0), DS3231_A2_Day); //Alarm when day (day of week), hours,inutes and seconds match 
    }else{
      rtc.clearAlarm(2);  //Alarm ausschalten
      noTone(4);  //Signalton ausschalten
    }




//TODO: muss wieder entfertn werden wnn wieder mit dem interupt gearbeitet wird


  static uint8_t anzeige_zeit;
  if(anzeige_zeit==150){
    if(page==0){
      displayTime (false);
    }

    if(rtc.alarmFired(1) && alarm1_ein_memory==true)
    {
      Signalgeber(true);
    }

    Serial.println("");
    Serial.println(alarm1_ein_memory);
  }
  anzeige_zeit++;

  //Wurde eine Geste erkannt
  Gestensensor();

  static uint8_t pause;
  if(modus==6){//Lichtabhängige Steuerung
    if(pause==200){
      //pause=0;
      bright=LDR_Messung();
      refreshColours();
    }
    pause++;
  }else if(modus==4){
    if(pause==200){
      //pause=0;
      partymodus();
    }
    pause++;
  }
}

void HMI_Input_loeschen(char* HMI_Input_array)
{
  for(int i=0; i<7;i++) //Inputdatenarray löschen
      {
        HMI_Input_array[i]="";
      }
}


//Interrupt Service Routine - This routine is performed when a falling edge on the 1Hz SQW clock from the RTC is detected
void ISR_RTC () {
    flanke_rtc_sqw = false; //A falling edge was detected on the SQWinput pin.  Now set EDGE equal to 0.
}


void displayTime (bool uhr_einstellen) {
  //TODO: übergabeparameter kann entfernt werden, kann immer angezeigt werden in den feldern
  DateTime now = rtc.now();
  char tag[6], monat[4], jahr[6], stunde[4], minute[4], sekunde[4], temperatur[8], w_tag[4];

  // Umwandlung der Zahlen in Char-Arrays
  sprintf(jahr, "%04d", now.year());
  sprintf(monat, "%02d", now.month());
  sprintf(tag, "%02d", now.day());
  sprintf(stunde, "%02d", now.hour());
  sprintf(minute, "%02d", now.minute());
  sprintf(sekunde, "%02d", now.second());
  strcpy(w_tag, wochentage[now.dayOfTheWeek()]);
  
  if(uhr_einstellen){
    Serielle_Textausgabe("u05.txt=",tag);
    Serielle_Textausgabe("u08.txt=",monat);
    Serielle_Textausgabe("u11.txt=",jahr);
    Serielle_Textausgabe("u14.txt=",stunde);
    Serielle_Textausgabe("u18.txt=",minute);
  }

  // Verkettung der Char-Arrays um Datum und die komplette Uhrzeit darzustellen
  sprintf(tag, "%s.%s.%s", tag, monat, jahr);
  sprintf(stunde, "%s:%s:%s", stunde, minute, sekunde);

  Serielle_Textausgabe("m01.txt=", w_tag);
  Serielle_Textausgabe("m02.txt=", tag);
  Serielle_Textausgabe("m03.txt=", stunde);
  dtostrf(rtc.getTemperature(), 6, 2, temperatur);
  sprintf(temperatur, "%s Grad C", temperatur);
  Serielle_Textausgabe("m04.txt=", temperatur);

  
}

boolean setModusActive(int newMod){
  if(modus==newMod){
    return 0;
  }
  //alten Modus abschalten
  switch(modus){
    case 1:
      sendValue("l01",0);
      break;
    case 2:
      sendValue("l02",0);
      break;
    case 3:
      sendValue("l03",0);
      break;
    case 4:
      sendValue("l05",0);
      break;
    case 6:
      sendValue("l04",0);
      break;
  }

  switch (newMod){
    case 1: //Lernen
      red=255;
      green=255;
      blue=255;
      bright=255;
      Serielle_Textausgabe("l06.txt=","Lernen"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      break;
    case 2: //Entspannungslicht (Relax)
      red=241;
      green=142;
      blue=28;
      bright=255;
      Serielle_Textausgabe("l06.txt=","Relax"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      break;
    case 4: //Party
      Serielle_Textausgabe("l06.txt=","Party"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
        //wird in anderen Funktion gehändelt
        //Task aktivieren?
      break;
    case 3: //Farben mix
      Serielle_Textausgabe("l06.txt=","Farbenmix"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      //activateHauptBel(mainRed,mainGreen,mainBlue,mainBright);
      //activateIndBel(indRed, indGreen, indBlue, indBright);
      break;
    case 6: //Lichtabhängige Lichtansteuerung (Automatik)
      red=255;
      green=255;
      blue=255;
      bright=LDR_Messung();
      Serielle_Textausgabe("l06.txt=","Automatik"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      break;
    default:
      break;
  }
  modus=newMod;
  return 1;
}

void refreshColours(){
  if(modus!=3){
    if(indirektebeleuchtung_an){
      activateIndBel(red, green, blue, bright);
    }else{
     activateIndBel(0,0,0,0);
    }
    if(hauptleuchte_an){
      activateHauptBel(red, green, blue, bright);
    }else{
      activateHauptBel(0,0,0,0);
    }
  }else{
    if(indirektebeleuchtung_an){
      activateIndBel(indRed, indGreen, indBlue, indBright);
    }else{
      activateIndBel(0,0,0,0);
    }
    if(hauptleuchte_an){
      activateHauptBel(mainRed, mainGreen, mainBlue, mainBright);
    }else{
      activateHauptBel(0,0,0,0);
    }
  }
  
}

void activateIndBel(uint8_t rot, uint8_t gruen, uint8_t blau, uint8_t hell){
  strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
  strip_IndLi.setBrightness(hell);
  strip_IndLi.show();
  strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
  strip_IndRe.setBrightness(hell);
  strip_IndRe.show();
}

void activateHauptBel(uint8_t rot, uint8_t gruen, uint8_t blau, uint8_t hell){
  main_light.fill(main_light.Color(rot, gruen, blau));
  main_light.setBrightness(hell);
  main_light.show();
}

void partymodus(){
  red=parRed[count%12];
  blue=parBlue[count%13];
  green=parGreen[count%14];
  bright=parBright[count%11];
  count++;
  refreshColours();
}

uint8_t LDR_Messung()
{
  uint8_t safe;
  helligkeit = analogRead(0); //Werte zwischen 0 und 1024
  //Serial.println("Helligkeit: "+helligkeit);
  //helligkeit=(helligkeit>800)?800:helligkeit;
  safe=255-helligkeit/4;
  //Serial.println("Safe: "+safe);
  return helligkeit;
}

void Serielle_Textausgabe(const char* textbox, const char* text)
{
  const char* cmd="\"";
  for(int i=0;i<2;i++){
      Serial.print(textbox);
      Serial.print(cmd);
      Serial.print(text);
      Serial.print(cmd);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
  }
}

void sendValue(const char* object, uint8_t value){
  char buf[3];
  sprintf(buf, "%02d", value);
  for(int i=0;i<2;i++){
      Serial.print(object);
      Serial.print(".val=");
      Serial.print(buf);
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(0xFF);
  }
}


uint8_t Gestensensor()
{
  Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
  gesture = sensor.readGesture();   // Read back current gesture (if any) of type Gesture

  switch (gesture)
  {
    case GES_FORWARD:
      {
        //deactivate Alarm

        break;
      }

    case GES_BACKWARD:
      {
        //empty
        break;
      }

    case GES_LEFT:
      {
        //empty
        break;
      }

    case GES_RIGHT:
      {
        //empty
        break;
      }

    case GES_UP:
      {
        //heller
        if(modus==3){
          mainBright=(mainBright>205)?255:mainBright+50;
          indBright=(indBright>205)?255:indBright+50;
        }else{
          bright=bright + 50;
          if(bright > 255){
            bright=255;
          }
        }
        refreshColours();
        break;
      }

    case GES_DOWN:
      {
        //dunkler
        if(modus==3){
          if(mainBright < 60){
            mainBright=10;
          }else{
            mainBright = mainBright -50;
          }
          if(indBright < 60){
            indBright=10;
          }else{
            indBright = indBright -50;
          }
        }else{
          if(bright < 60){
            bright=10;
          }else{
            bright = bright -50;
          }
        }
        refreshColours();
        break;
      }

    case GES_CLOCKWISE:
      {
        //empty
        break;
      }

    case GES_ANTICLOCKWISE:
      {
        //empty
        break;
      }

    case GES_WAVE:
      {
        //Licht toggeln
          if(hauptleuchte_an||indirektebeleuchtung_an){
            hauptleuchte_an=false;
            indirektebeleuchtung_an=false;
          }else{
            hauptleuchte_an=true;
            indirektebeleuchtung_an=true;
          }
          refreshColours();
        break;
      }

    case GES_NONE:
      {
        break;
      }
  }
  return 1;
}

void Signalgeber(bool ton_an)
{
  if(ton_an==true){
    tone(4,264); // (pin, frequency, duration)
    delay(100);
    tone(4,547);
    delay(100);
  }else{
    noTone(4);
  }
}