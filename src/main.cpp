// 19.12.2023 RAM: 39,6%  Flash: 42,1%
// 20.12.2023 RAM: 33,3%  Flash: 40,7%
// 23.12.2023 RAM: 36,8%  Flash: 39,3% (Uhrzeit implementiert)

#include <Arduino.h>
#include <avr/power.h>

#include "Adafruit_NeoPixel.h"  //Indirekte Beleuchtung
#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include "RevEng_PAJ7620.h"  //Gestensensor
#include "RTClib.h"  //Realtime-Clock

// Variablen:
#define LED_PIN_IndLi    6    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndLi 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define LED_PIN_IndRe    5    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndRe 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define main_light_count 25
#define main_light_pin 3
#define PIN_schalter 7
#define PIN_summer 4 //Pin an dem der Summer angeschlossen ist

uint16_t helligkeit; //Wird benötigt für die LDR-Messung
uint8_t modNeu=0;
uint8_t parRed[12]={128, 192, 239, 255, 239, 192, 128, 64, 17, 0, 17, 64};
uint8_t parBlue[13]={213, 159, 97, 43, 8, 1, 23, 69, 128, 187, 233, 255, 248};
uint8_t parGreen[14]={2, 5, 32, 78, 134, 189, 232, 254, 251, 224, 178, 122, 67, 24};
uint8_t parBright[11]={128, 169, 196, 202, 185, 149, 107, 71, 54, 60, 87};
uint8_t count=0;
//int pause=0;
uint8_t page=0; //0 Main-Seite, 1 Lichtkonfiguration, 2 Wecker, 3 Einstellungen, 4 Gesten, 5 Uhreinstellungen, 6 LichtSpezial, 7 Gesten Nr.2, 8 Alarm aktiv
bool gestureActive=true; //Zeigt ob der Gestensensor Eingaben verarbeiten kann
bool settingAlarm=false;
//uint8_t mem_day=0;

//Speicher für den Mixmodus
uint8_t indRed,indGreen,indBlue,indBright;
uint8_t mainRed,mainGreen,mainBlue,mainBright;
uint32_t memory;
uint8_t modus=0;  //1 Lernen, 2 Relax, 3 Mix, 4 Party, 6 Auto
bool hauptleuchte_an=true;
bool indirektebeleuchtung_an=true;
uint8_t activeLamp=1; //0 beide aus; 1 Haupt; 2 Neben
uint8_t red,green,blue,bright;

char printHelp[13];
char tag[6], monat[4], jahr[6], stunde[4], minute[4], sekunde[4], temperatur[8], w_tag[4];
char sendingBuffer[20];

bool alarm1Woche[7] = {true, true, true, true, true, true, true};
bool alarm1_ein_memory=false;
//bool alarm2_ein_memory=false;
int8_t alarm1_minute_memory;
int8_t alarm1_stunde_memory;
//int8_t alarm2_minute_memory;
//int8_t alarm2_stunde_memory;
bool alarm_fired=false;
int8_t alarm1_timeout;
//int8_t alarm2_timeout;

//TODO: Können die Bool-Werte nicht einfacher in zwei Integern gespeichert werden? Wäre auch leichter zu verstehen.
char hmi_input [7]={};    //Es werden 4 char benötigt, da wir 4 Datensätze pro Button übertragen, um diesen zu identifizieren

// Funktionen:
void Signalgeber(bool); //(An/Aus)
uint8_t Gestensensor(); //Gestensensor
uint8_t LDR_Messung(); //LDR Messung zwischen 0 und 1023
void sentTextState(const char*, const char*); //Textausgabe zum HMI
void DisplayCommand(const char*);
void clearCharArray(const char*);
void displayTime (bool); //Ausgabe der aktuellen Zeit
void HMI_Input_loeschen(char*);
boolean setModusActive(uint8_t);
void partymodus();
void refreshColours();
void activateIndBel(uint8_t, uint8_t, uint8_t, uint8_t);
void activateHauptBel(uint8_t, uint8_t, uint8_t, uint8_t);
void sendValue(const char*, uint8_t);
void getSliderValue(const char*);
void alarmOccured(uint8_t);
void stopAlarm();

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
const char roterSlider[4]="p01";
const char gruenerSlider[4]="p03";
const char blauerSlider[4]="p05";
const char hellerSlider[4]="p07";
const char lichtText[4]="l06";
const char textFeldTag[4]="u05";
const char textFeldMonat[4]="u08";
const char textFeldJahr[4]="u11";
const char textFeldStunde[4]="u14";
const char textFeldMinute[4]="u18";
bool requestedRed=false;
bool requestedGreen=false;
bool requestedBlue=false;
bool requestedBright=false;
const char convertToNumber[5]="%02d";

//Die Farbwerte für den aktuellen Modus speichern
//Speichert die Werte für die aktiven Lampen
void bt_save_lsPopCallback(){
  if(activeLamp==0){
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
  activeLamp++;
  switch(activeLamp){
    case 1: 
        hauptleuchte_an=true;
        indirektebeleuchtung_an=false;
        red=mainRed;
        blue=mainBlue;
        green=mainGreen;
        bright=mainBright;
        break;
    case 2: 
        hauptleuchte_an=false;
        indirektebeleuchtung_an=true;
        red=indRed;
        blue=indBlue;
        green=indGreen;
        bright=indBright;
        activeLamp=0;
        break;
    default: 
        break;
  }
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
  sendValue(hellerSlider,bright);
  //delay(10);
  sendValue(roterSlider,red);
  //delay(10);
  sendValue(blauerSlider,blue);
  //delay(10);
  sendValue(gruenerSlider,green);
  refreshColours();
}

//Wecker + Uhrzeit
RTC_DS3231 rtc;
char wochentage[7][3] = {"So","Mo", "Di", "Mi", "Do", "Fr", "Sa"};


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
  
  //Serial.println("Recognizing gestures");

  if( !sensor.begin() )           // return value of 0 == success
  {
    //Serial.println("PAJ7620 I2C error");
  }else{
    //Serial.println("PAJ7620 init OK");
  }
  
  //RTC
  if (! rtc.begin()) {
    //TODO: Ausgabe der Fehlermeldung auf Touchdisplay
    //Serial.println("RTC nicht gefunden");
    Serial.flush();
    abort();
  }else{
    //Serial.println("RTC initialisiert");
  }

   rtc.disable32K();

  if (rtc.lostPower()) {
    //TODO: Implementieren auf Touchdisplay evtl. sichtbar
     //Serial.println("RTC lost power, set the time");
  }
  //Alarm (Wecker)
  rtc.disableAlarm(1);
  //rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  //rtc.clearAlarm(2);  
  rtc.writeSqwPinMode(DS3231_OFF);
  setModusActive(1);
}


// Main-Code-Schleife, diese Methode wird ständig wiederholt
void loop() 
{
  DateTime now = rtc.now();

  //HMI input bitstream lesen
  if(Serial.available() > 0) //Prüfe ob Serielle Schnittstelle erreichbar
  {
    for(uint8_t i=0;i<6;i++) //Eingehende Nummer von Inputs einlesen (xx yy zz dd aa uu ii) yy=Seite, zz=Button 1,2,3
    {
      hmi_input[i]=hmi_input[i+1];
    }
    hmi_input[6]=Serial.read();
  }


if(hmi_input[0]==0x65){
//Äußere switch seite
//switch in Case von äußerem switch button
//hmi_input[1]=> Seite 2, hmi_input[2] => Button
switch (hmi_input[1])
{
  case 0x00: //Mainpage
      switch(hmi_input[2]){
        case 2://Licht Einstellungen
          page = 1;
          //Radio-Buttons, funktioniert nicht
          //TODO: 
          
          if(!hauptleuchte_an){
            sendValue("l07",0);
          }
          if(!indirektebeleuchtung_an){
            sendValue("l08",0);
          }          
          break;
        case 1:
          page = 2;
          break;
        case 3:
          page = 3;
          break;
        case 12:
          //Licht toggeln
          if(hauptleuchte_an||indirektebeleuchtung_an){
            hauptleuchte_an=false;
            indirektebeleuchtung_an=false;
          }else{
            hauptleuchte_an=true;
            indirektebeleuchtung_an=true;
          }
          refreshColours();
          refreshColours();
          break;
        
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
          hauptleuchte_an=true;
          indirektebeleuchtung_an=false;
          setModusActive(5);
          page=6;
          break;
        case 0x09://Hauptlicht an?
          hauptleuchte_an=!hauptleuchte_an;
          refreshColours();
          break;
        case 0x0A://Nebenlicht an?
          indirektebeleuchtung_an=!indirektebeleuchtung_an;
          refreshColours();
          break;
      }
      HMI_Input_loeschen(hmi_input);
  break;

  case 0x02: //Wecker-page
      //char alarm1_stunde[4];
      //char alarm1_minute[4];
      //char alarm2_stunde[4];
      //char alarm2_minute[4];
      
        switch (hmi_input[2])
        {
          case 0x02:
            page=0;
            break;
          case 0x13: //Alarm 1Stunde verringern(1, 0, 23,...)
            --alarm1_stunde_memory;
            if(alarm1_stunde_memory<0){
              alarm1_stunde_memory=23;
            }
            sprintf(stunde, convertToNumber, alarm1_stunde_memory);
            sentTextState("a10",stunde);
            break;
          case 0x11: //Alarm 1 Stunde erhöhen (1, 2, 3,...)
            ++alarm1_stunde_memory;
            if(alarm1_stunde_memory>23){
            alarm1_stunde_memory=0;
            }
            sprintf(stunde, convertToNumber, alarm1_stunde_memory);
            sentTextState("a10",stunde);
            break; 

          case 0x16: //Alarm 1 Minute verringern(1, 59, 58,...)
            --alarm1_minute_memory;
            if(alarm1_minute_memory<0){
            alarm1_minute_memory=59;
            }
            sprintf(minute, convertToNumber, alarm1_minute_memory);
            sentTextState("a12",minute);
            break;
          case 0x14: //Alarm 1 Minute erhöhen (1, 2, 3,...)
            ++alarm1_minute_memory;
            if(alarm1_minute_memory>59){
            alarm1_minute_memory=0;
            }
            sprintf(minute, convertToNumber, alarm1_minute_memory);
            sentTextState("a12",minute);
            break;

          case 0x03: //Montag (Alarm1)
              alarm1Woche[1]=!alarm1Woche[1];
              //montag_alarm1_memory=!montag_alarm1_memory; 
            break;
          case 0x04: //Dienstag (Alarm1)
              alarm1Woche[2]=!alarm1Woche[2];
              //dienstag_alarm1_memory=!dienstag_alarm1_memory;    
            break;
          case 0x06: //Mittwoch (Alarm1)
              alarm1Woche[3]=!alarm1Woche[3];
              //mittwoch_alarm1_memory=!mittwoch_alarm1_memory; 
            break;
          case 0x05: //Donnerstag (Alarm1)
              alarm1Woche[4]=!alarm1Woche[4];
              //donnerstag_alarm1_memory=!donnerstag_alarm1_memory;
            break;
          case 0x08: //Freitag (Alarm1)
              alarm1Woche[5]=!alarm1Woche[5];
              //freitag_alarm1_memory=!freitag_alarm1_memory;
            break;
          case 0x07:  //Samstag (Alarm1)
              alarm1Woche[6]=!alarm1Woche[6];
              //samstag_alarm1_memory=!samstag_alarm1_memory;
            break;
          case 0x09: //Sonntag (Alarm1)
              alarm1Woche[0]=!alarm1Woche[0];
              //sonntag_alarm1_memory=!sonntag_alarm1_memory;
            break;

          /*
          case 0x1B: //Alarm 2 Stunde verringern(1, 0, 23,...)
            --alarm2_stunde_memory;
            if(alarm2_stunde_memory<0){
            alarm2_stunde_memory=23;
            }
            sprintf(alarm2_stunde, convertToNumber, alarm2_stunde_memory);
            sentTextState("a17",alarm2_stunde);
            break;
          case 0x1D: //Alarm 2 Stunde erhöhen (1, 2, 3,...)
            ++alarm2_stunde_memory;
            if(alarm2_stunde_memory>23){
            alarm2_stunde_memory=0;
            }
            sprintf(alarm2_stunde, convertToNumber, alarm2_stunde_memory);
            sentTextState("a17",alarm2_stunde);
            break; 

          case 0x18: //Alarm 2 Minute verringern(1, 59, 58,...)
            --alarm2_minute_memory;
            if(alarm2_minute_memory<0){
            alarm2_minute_memory=59;
            }
            sprintf(alarm2_minute, convertToNumber, alarm2_minute_memory);
            sentTextState("a19",alarm2_minute);
            break;
          case 0x1A: //Alarm 2 Minute erhöhen (1, 2, 3,...)
            ++alarm2_minute_memory;
            if(alarm2_minute_memory>59){
            alarm2_minute_memory=0;
            }
            sprintf(alarm2_minute, convertToNumber, alarm2_minute_memory);
            sentTextState("a19",alarm2_minute);
            break;

          case 0x0A: //Montag (Alarm2)
              montag_alarm2_memory=!montag_alarm2_memory;
            break;
          case 0x0B: //Dienstag (Alarm2)
              dienstag_alarm2_memory=!dienstag_alarm2_memory;     
            break;
          case 0x0D: //Mittwoch (Alarm2)
              mittwoch_alarm2_memory=!mittwoch_alarm2_memory;
            break;
          case 0x0C: //Donnerstag (Alarm2)
              donnerstag_alarm2_memory=!donnerstag_alarm2_memory;
            break;
          case 0x0F: //Freitag (Alarm2)
              freitag_alarm2_memory=!freitag_alarm2_memory; 
            break;
          case 0x0E:  //Samstag (Alarm2)
              samstag_alarm2_memory=!samstag_alarm2_memory;
            break;
          case 0x10: //Sonntag (Alarm2)
              sonntag_alarm2_memory=!sonntag_alarm2_memory;
            break;
          */

          case 0x23: //Alarm 1 Ein/Aus
              if(alarm1_ein_memory==true)
              {
                alarm1_ein_memory=false;
              } else{
                alarm1_ein_memory=true;
                alarm1_timeout = alarm1_minute_memory + 3;
                if(alarm1_timeout>59){
                  alarm1_timeout = alarm1_timeout - 60;
                }
              } 
            break;
          /*
          case 0x24: //Alarm 2 Ein/Aus
              if(alarm2_ein_memory==true)
              {
                rtc.clearAlarm(2);
                alarm2_ein_memory=false;
              } else if (alarm2_ein_memory==false)
              {
                alarm2_ein_memory=true;
                alarm2_timeout = alarm2_minute_memory + 3;
                if(alarm2_timeout>59){
                  alarm2_timeout = alarm2_timeout - 60;
                }
              } 
            break;
          */
            default:
              break;
        }
        HMI_Input_loeschen(hmi_input);
        if(alarm1_ein_memory){
          rtc.setAlarm1(DateTime(now.year(), now.month(), now.day(), alarm1_stunde_memory, alarm1_minute_memory, 0), DS3231_A1_Hour); //Alarm when day (day of week), hours,inutes and seconds match 
        }else{
          rtc.disableAlarm(1);
        }
  break;

  case 0x03: //Settings-page
      switch (hmi_input[2])
      {
        case 0x02: //Home-Button
          page=0;
          break;
        case 0x03: //Uhr Einstellungen
          displayTime(true);
          break;
        case 0x06:  //Gestensensor aktiv?
          gestureActive=!gestureActive;
          if(gestureActive){
            sensor.enable();
          }else{
            sensor.disable();
          }

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
      static int8_t tag_memory=now.day();
      //char tag[3];
      static int8_t monat_memory=now.month();
      //char monat[3];
      static int16_t jahr_memory=now.year();
      //char jahr[6];
      static int8_t stunde_memory=now.hour();
      //char stunde[4];
      static int8_t minute_memory=now.minute();
      //char minute[4];

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
        sprintf(tag, convertToNumber, tag_memory);
        sentTextState(textFeldTag,tag);
        break;
      case 0x0A: //Tag erhöhen (1, 2, 3,...)
        tag_memory++;
        if(tag_memory>31){
        tag_memory=1;
        }
        sprintf(tag, convertToNumber, tag_memory);
        sentTextState(textFeldTag,tag);
        break;

      case 0x0F: //Monat verringern(1, 12, 1,...)
        monat_memory--;
        if(monat_memory<1){
        monat_memory=12;
        }
        sprintf(monat, convertToNumber, monat_memory);
        sentTextState(textFeldMonat,monat);
        break;
      case 0x0D: //Monat erhöhen (1, 2, 3,...)
        monat_memory++;
        if(monat_memory>12){
        monat_memory=1;
        }
        sprintf(monat, convertToNumber, monat_memory);
        sentTextState(textFeldMonat,monat);
        break;

      case 0x12: //Jahr verringern
        jahr_memory--;
        if(jahr_memory<1){
        jahr_memory=9999;
        }
        sprintf(jahr, "%04d", jahr_memory);
        sentTextState(textFeldJahr,jahr);
        break;
      case 0x10: //Jahr erhöhen 
        jahr_memory++;
        if(jahr_memory>9999){
        jahr_memory=1;
        }
        sprintf(jahr, "%04d", jahr_memory);
        sentTextState(textFeldJahr,jahr);
        break;

      case 0x15: //Stunde verringern(1, 0, 23,...)
        stunde_memory--;
        if(stunde_memory<0){
        stunde_memory=23;
        }
        sprintf(stunde, convertToNumber, stunde_memory);
        sentTextState(textFeldStunde,stunde);
        break;
      case 0x13: //Stunde erhöhen (1, 2, 3,...)
        stunde_memory++;
        if(stunde_memory>23){
        stunde_memory=0;
        }
        sprintf(stunde, convertToNumber, stunde_memory);
        sentTextState(textFeldStunde,stunde);
        break; 

      case 0x18: //Minute verringern(1, 59, 58,...)
        minute_memory--;
        if(minute_memory<1){
        minute_memory=59;
        }
        sprintf(minute, convertToNumber, minute_memory);
        sentTextState(textFeldMinute,minute);
        break;
      case 0x16: //Minute erhöhen (1, 2, 3,...)
        minute_memory++;
        if(minute_memory>59){
        minute_memory=0;
        }
        sprintf(minute, convertToNumber, minute_memory);
        sentTextState(textFeldMinute,minute);
        break;

      case 0x04: //Speichern
        rtc.adjust(DateTime(jahr_memory, monat_memory, tag_memory, stunde_memory, minute_memory, 0)); //(Jahr, Monat, Tag, Stunde, Minute, Sekunde)
        break;

      default:
        break;
      }
      HMI_Input_loeschen(hmi_input);
      break;

  case 0x06: //Farbmix-page
      switch(hmi_input[2]){
        case 0x02:
          bt_save_lsPopCallback();
          break;
        case 0x0E:
          b_switch_lsPopCallback();
          break;
        case 0x01:
          getSliderValue(roterSlider);
          requestedRed=true;
          break;
        case 0x04:
          getSliderValue(gruenerSlider);
          requestedGreen=true;
          break;
        case 0x06:
          getSliderValue(blauerSlider);
          requestedBlue=true;
          break;
        case 0x08:
          getSliderValue(hellerSlider);
          requestedBright=true;
          break;
        case 0x0F: //Home-Button
          page=0;
          setModusActive(3);
          sensor.clearGestureInterrupts();
          break;
      }
      HMI_Input_loeschen(hmi_input);
  break;

  case 7:
    if(hmi_input[2]==3){
      page=0;
    }
    HMI_Input_loeschen(hmi_input);
  break;

  case 8:
    stopAlarm();
    HMI_Input_loeschen(hmi_input);
    break;

  default:
    break;
  }
}else if(hmi_input[0]==0x71){//Zweite Klammer für 0x65
  //Das sind die Antworten auf unsere Get-Anfrage
  if(requestedRed){
    red=(uint8_t)hmi_input[1];
    requestedRed=false;
  }else if(requestedGreen){
    green=(uint8_t)hmi_input[1];
    requestedGreen=false;
  }else if(requestedBlue){
    blue=(uint8_t)hmi_input[1];
    requestedBlue=false;
  }else if(requestedBright){
    bright=(uint8_t)hmi_input[1];
    requestedBright=false;
  }
  refreshColours();
}

  

  static uint8_t anzeige_zeit;
  if(anzeige_zeit==150){
    //Sind wir auf der richtigen Seite, um die Zeit auszugeben?
    if(page==0){
      displayTime(false);
    }
    //Ist der Gestensensor aktiv?
    if(gestureActive){
      if(Gestensensor()==1){
        //Serial.println("Geste");
      }
    }
    // Ist der Alarm hochgegangen?
    if(rtc.alarmFired(1))
    {
      if(alarm1Woche[now.dayOfTheWeek()]==true){
        alarmOccured(now.minute());
      }else{
        rtc.clearAlarm(1);
      }
    }

    //Ist der Automatische Modus aktiv?
    if(modus==6){//Lichtabhängige Steuerung
      bright=LDR_Messung();
      refreshColours();
    }else if(modus==4){ //Ist der Partymodus aktiv?
      partymodus();
    }
    Serial.println("");
  }
  anzeige_zeit++;
}//++++++++++++++++++++++  Ende von loop  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void HMI_Input_loeschen(char* HMI_Input_array)
{
  for(uint8_t i=0; i<7;i++) //Inputdatenarray löschen
      {
        HMI_Input_array[i]=0;
      }
}

void displayTime (bool uhr_einstellen) {
  //TODO: übergabeparameter kann entfernt werden, kann immer angezeigt werden in den feldern
  DateTime now = rtc.now();

  // Umwandlung der Zahlen in Char-Arrays
  sprintf(jahr, "%04d", now.year());
  sprintf(monat, convertToNumber, now.month());
  sprintf(tag, convertToNumber, now.day());
  sprintf(stunde, convertToNumber, now.hour());
  sprintf(minute, convertToNumber, now.minute());
  sprintf(sekunde, convertToNumber, now.second());
  strcpy(w_tag, wochentage[now.dayOfTheWeek()]);
  
  if(uhr_einstellen){
    sentTextState("u05",tag);
    sentTextState("u08",monat);
    sentTextState("u11",jahr);
    sentTextState("u14",stunde);
    sentTextState("u18",minute);
  }else{
    // Verkettung der Char-Arrays um Datum und die komplette Uhrzeit darzustellen
    sentTextState("m01", w_tag);
    
    sprintf(printHelp, " %s.%s.%s ", tag, monat, jahr);
    sentTextState("m02", printHelp);

    sprintf(printHelp, "  %s:%s:%s  ", stunde, minute, sekunde);
    sentTextState("m03", printHelp);

    dtostrf(rtc.getTemperature(), 6, 2, temperatur);
    sprintf(printHelp, "%s Grad C", temperatur);
    sentTextState("m04", printHelp);
  }
  
}

boolean setModusActive(uint8_t newMod){
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
      sentTextState(lichtText,"Lernen"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 2: //Entspannungslicht (Relax)
      red=241;
      green=142;
      blue=28;
      bright=255;
      sentTextState(lichtText,"Relax"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 4: //Party
      sentTextState(lichtText,"Party"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 3: //Farben mix
      sentTextState(lichtText,"Farbenmix"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      break;
    case 6: //Lichtabhängige Lichtansteuerung (Automatik)
      sentTextState(lichtText,"Automatik"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      red=255;
      green=255;
      blue=255;
      break;
    case 5: //Lichtkonfig-Seite
      red=0;
      green=0;
      blue=0;
      bright=0;
    default:
      break;
  }
  modus=newMod;
  refreshColours();
  delay(10);
  refreshColours();
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
  
  helligkeit = analogRead(0); //Werte zwischen 0 und 1024
  if(page==1){
    char val[3];
    sprintf(val, convertToNumber, helligkeit);
    sentTextState(lichtText,val);
  }
  
  //TODO: Bereiche anpassen, indem die Helligkeit gemessen wird.
  /*  //Range für Fabios Sensor
  if(helligkeit>300){
    return 240;
  }else if(helligkeit>200){
    return 170;
  }else if(helligkeit>150){
    return 130;
  }else if(helligkeit>100){
    return 90;
  }else if(helligkeit>50){
    return 50;
  }else{
    return 0;
  }
  */
    //Range für Flos Sensor
  if(helligkeit>900){
    return 0;
  }else if(helligkeit>800){
    return 50;
  }else if(helligkeit>730){
    return 90;
  }else if(helligkeit>630){
    return 130;
  }else if(helligkeit>550){
    return 50;
  }else{
    return 0;
  }
}

uint8_t Gestensensor()
{
  Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
  gesture = sensor.readGesture();   // Read back current gesture (if any) of type Gesture
  sensor.clearGestureInterrupts();

  switch (gesture)
  {
    case GES_NONE:{return 0;}
    case GES_FORWARD:
      {
        //deactivate Alarm
        if(rtc.alarmFired(1)){
          stopAlarm();
          return 1;
        }
        break;
      }

    case GES_BACKWARD:{break;}
    case GES_LEFT:{break;}

    case GES_RIGHT:
      {
        if(page==1||page==6){
          return 0;
        }
        //Licht toggeln
        if(hauptleuchte_an||indirektebeleuchtung_an){
          hauptleuchte_an=false;
          indirektebeleuchtung_an=false;
        }else{
          hauptleuchte_an=true;
          indirektebeleuchtung_an=true;
        }
        refreshColours();
        refreshColours();
        return 1;
      }

    case GES_UP:
      {
        if(page==1||page==6){
          return 0;
        }
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
        return 1;
      }

    case GES_DOWN:
      {
        if(page==1||page==6){
          return 0;
        }
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
        return 1;
      }
    case GES_CLOCKWISE:{break;}
    case GES_ANTICLOCKWISE:{break;}
    case GES_WAVE:{break;}
  }
  return 0;
}

void Signalgeber(bool ton_an)
{
  if(ton_an==true){
    //uint8_t pulse=1;
    tone(4,264); // (pin, frequency, duration)
    //while(pulse!=0){pulse++;    }
    delay(100);
    //pulse=1;
    tone(4,547);
    //while(pulse!=0){pulse++;}
    delay(100);
  }else{
    noTone(4);
    delay(300);
  }
}

void sentTextState(const char* textbox, const char* text)
{
  memset(&sendingBuffer[0], 0, sizeof(sendingBuffer));
  sprintf(sendingBuffer, "%s%s%s%s", textbox, ".txt=\"", text,"\"");
  DisplayCommand(sendingBuffer);
}

void DisplayCommand(const char* textbox)
{
  for(uint8_t i=0;i<2;i++){
    Serial.print(textbox);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
  }
}

void sendValue(const char* object, uint8_t value){
  char buf[3];
  sprintf(buf, convertToNumber, value);
  memset(&sendingBuffer[0], 0, sizeof(sendingBuffer));
  sprintf(sendingBuffer, "%s%s%s", object, ".val=", buf);
  DisplayCommand(sendingBuffer);
  //Serial.println("");
}

void getSliderValue(const char* object){
  memset(&sendingBuffer[0], 0, sizeof(sendingBuffer));
  sprintf(sendingBuffer, "%s%s%s", "get ",object, ".val");
  DisplayCommand(sendingBuffer);
}

void alarmOccured(uint8_t actualMinute){
  if(!alarm_fired){
    DisplayCommand("page 8");
    page=8;
    alarm_fired=true;
  }
  Signalgeber(true);
  if(actualMinute==alarm1_timeout){
    stopAlarm();    
  }
}

void stopAlarm(){
  rtc.clearAlarm(1);
  alarm_fired=false;
  DisplayCommand("page 0");
  Signalgeber(false);
  page=0;
}