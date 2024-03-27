// 19.12.2023 RAM: 39,6%  Flash: 42,1%
// 20.12.2023 RAM: 33,3%  Flash: 40,7%
// 23.12.2023 RAM: 36,8%  Flash: 39,3% (Uhrzeit implementiert)
// 31.12.2023 RAM: 44,6%  Flash: 48,5%

//TODO: Wenn Alarm im Display verändert wird muss dies automatisch angepasst werden oder es muss erneut gespeichert werden

#include <Arduino.h>
#include "Adafruit_NeoPixel.h"  //Beleuchtung
#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include "RevEng_PAJ7620.h"  //Gestensensor
#include "RTClib.h"  //Realtime-Clock

// Variablen:
#define LED_PIN_IndLi    6    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndLi 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define LED_PIN_IndRe    5    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndRe 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define main_light_width 5
#define main_light_high 5
#define main_light_pin 3
#define PIN_LED 13
#define PIN_schalter 7
#define PIN_summer 4 //Pin an dem der Summer angeschlossen ist
#define PIN_SQW 2
//const byte PIN_SQW = 2; //Interruptpin RTC -> SQW pin is used to monitor the SQW 1Hz output from the DS3231
//bool flanke_rtc_sqw; //A variable to store when a falling 1Hz clock edge has occured on the SQW pin of the DS3231
uint8_t helligkeit; //Wird benötigt für die LDR-Messung
bool flanke_Licht_ein = false;
uint8_t modus=0;  //1 Lernen, 2 Relax, 3 Mix, 4 Party, 6 Auto
uint8_t leuchtstaerke=0;
bool hauptleuchte_an=false;
bool indirektebeleuchtung_an=false;
int durchlaufzaehler_party_farbwechsel=0;
volatile uint8_t flankenzaehler_ein_aus=0;
uint8_t activeLamp=0; //0 beide aus; 1 Haupt; 2 Neben; 3 beide
uint8_t red,green,blue,bright;
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
bool alarm_fired=false;

// Funktionen:
uint8_t RGB_Licht_Funktion(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, bool, bool); //(Pixel,R,G,B,Helligkeit,Modus,Hauptleuchte_an,Indirektebeleuchtung_an)
void Signalgeber(bool); //(An/Aus)
uint8_t Gestensensor(); //Gestensensor
int LDR_Messung(); //LDR Messung zwischen 0 und 1023
void Serielle_Textausgabe(const char*, const char*); //Textausgabe zum HMI
void Serielle_Textausgabe2(const char*); //Textausgabe zum HMI
void displayTime (bool, DateTime); //Ausgabe der aktuellen Zeit
void HMI_Input_loeschen(char*);

// Objekte:
Adafruit_NeoPixel strip_IndLi(LED_COUNT_IndLi, LED_PIN_IndLi, NEO_GRB + NEO_KHZ800);    // NeoPixel pixel object:
Adafruit_NeoPixel strip_IndRe(LED_COUNT_IndRe, LED_PIN_IndRe, NEO_GRB + NEO_KHZ800);    // NeoPixel pixel object:
Adafruit_NeoPixel main_light(25,main_light_pin, NEO_GRB + NEO_KHZ800);

//Gestensensorobjekt
RevEng_PAJ7620 sensor = RevEng_PAJ7620();

//Wecker + Uhrzeit
RTC_DS3231 rtc;
char wochentage[7][3] = {"So","Mo", "Di", "Mi", "Do", "Fr", "Sa"};


// Diese Mehtode wird einmalig zum Programmstart ausgeführt.
void setup() 
{

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
 if( !sensor.begin() )   
  {
    Serial.print("Gestensensor error");
  } 

  //RTC
  if (! rtc.begin()) {
    //TODO: Ausgabe der Fehlermeldung auf Touchdisplay
    Serial.println("RTC nicht gefunden");
    Serial.flush();
  }
   rtc.disable32K();
  if (rtc.lostPower()) {
    //TODO: Implementieren auf Touchdisplay evtl. sichtbar
     Serial.println("RTC lost power!");
  }

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
  case 0: //Mainpage
        HMI_Input_loeschen(hmi_input);
  break;

  case 1: //Licht-Konfig-page
      HMI_Input_loeschen(hmi_input);
  break;

  case 2: //Wecker-page
      char alarm1_stunde[4];
      char alarm1_minute[4];
      char alarm2_stunde[4];
      char alarm2_minute[4];
      
        switch (hmi_input[2])
        {
          case 0x13: //Alarm 1Stunde verringern(1, 0, 23,...)
            --alarm1_stunde_memory;
            if(alarm1_stunde_memory<0){
              alarm1_stunde_memory=23;
            }
            sprintf(alarm1_stunde, "%02d", alarm1_stunde_memory);
            Serielle_Textausgabe("a10.txt=",alarm1_stunde);
            break;
          case 0x11: //Alarm 1 Stunde erhöhen (1, 2, 3,...)
            ++alarm1_stunde_memory;
            if(alarm1_stunde_memory>23){
            alarm1_stunde_memory=0;
            }
            sprintf(alarm1_stunde, "%02d", alarm1_stunde_memory);
            Serielle_Textausgabe("a10.txt=",alarm1_stunde);
            break; 

          case 0x16: //Alarm 1 Minute verringern(1, 59, 58,...)
            --alarm1_minute_memory;
            if(alarm1_minute_memory<0){
            alarm1_minute_memory=59;
            }
            sprintf(alarm1_minute, "%02d", alarm1_minute_memory);
            Serielle_Textausgabe("a12.txt=",alarm1_minute);
            break;
          case 0x14: //Alarm 1 Minute erhöhen (1, 2, 3,...)
            ++alarm1_minute_memory;
            if(alarm1_minute_memory>59){
            alarm1_minute_memory=0;
            }
            sprintf(alarm1_minute, "%02d", alarm1_minute_memory);
            Serielle_Textausgabe("a12.txt=",alarm1_minute);
            break;

          case 0x03: //Montag (Alarm1)
              montag_alarm1_memory=!montag_alarm1_memory; 
            break;
          case 0x04: //Dienstag (Alarm1)
              dienstag_alarm1_memory=!dienstag_alarm1_memory;    
            break;
          case 0x06: //Mittwoch (Alarm1)
              mittwoch_alarm1_memory=!mittwoch_alarm1_memory; 
            break;
          case 0x05: //Donnerstag (Alarm1)
              donnerstag_alarm1_memory=!donnerstag_alarm1_memory;
            break;
          case 0x08: //Freitag (Alarm1)
              freitag_alarm1_memory=!freitag_alarm1_memory;
            break;
          case 0x07:  //Samstag (Alarm1)
              samstag_alarm1_memory=!samstag_alarm1_memory;
            break;
          case 0x09: //Sonntag (Alarm1)
              sonntag_alarm1_memory=!sonntag_alarm1_memory;
            break;

          case 0x1B: //Alarm 2 Stunde verringern(1, 0, 23,...)
            --alarm2_stunde_memory;
            if(alarm2_stunde_memory<0){
            alarm2_stunde_memory=23;
            }
            sprintf(alarm2_stunde, "%02d", alarm2_stunde_memory);
            Serielle_Textausgabe("a17.txt=",alarm2_stunde);
            break;
          case 0x1D: //Alarm 2 Stunde erhöhen (1, 2, 3,...)
            ++alarm2_stunde_memory;
            if(alarm2_stunde_memory>23){
            alarm2_stunde_memory=0;
            }
            sprintf(alarm2_stunde, "%02d", alarm2_stunde_memory);
            Serielle_Textausgabe("a17.txt=",alarm2_stunde);
            break; 

          case 0x18: //Alarm 2 Minute verringern(1, 59, 58,...)
            --alarm2_minute_memory;
            if(alarm2_minute_memory<0){
            alarm2_minute_memory=59;
            }
            sprintf(alarm2_minute, "%02d", alarm2_minute_memory);
            Serielle_Textausgabe("a19.txt=",alarm2_minute);
            break;
          case 0x1A: //Alarm 2 Minute erhöhen (1, 2, 3,...)
            ++alarm2_minute_memory;
            if(alarm2_minute_memory>59){
            alarm2_minute_memory=0;
            }
            sprintf(alarm2_minute, "%02d", alarm2_minute_memory);
            Serielle_Textausgabe("a19.txt=",alarm2_minute);
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

          case 0x23: //Alarm 1 Ein/Aus
              if(alarm1_ein_memory==true)
              {
                rtc.clearAlarm(1);
                alarm1_ein_memory=false;
              } else if (alarm1_ein_memory==false)
              {
                alarm1_ein_memory=true;
              } 
            break;
          case 0x24: //Alarm 2 Ein/Aus
              if(alarm2_ein_memory==true)
              {
                rtc.clearAlarm(2);
                alarm2_ein_memory=false;
              } else if (alarm2_ein_memory==false)
              {
                alarm2_ein_memory=true;
              } 
            break;

            default:
              break;
        }
        HMI_Input_loeschen(hmi_input); 
  break;

  case 3: //Settings-page
      switch (hmi_input[2])
      {
      case 0x03:
        displayTime(true, now);
        break;
      
      default:
        break;
      }
      HMI_Input_loeschen(hmi_input);
  break;

  case 4: //Gestensteuerung-page
      HMI_Input_loeschen(hmi_input);
  break;

  case 5: //Uhr-Konfig-page
      static int8_t tag_memory=now.day();
      char tag[3];
      static int8_t monat_memory=now.month();
      char monat[3];
      static int16_t jahr_memory=now.year();
      char jahr[5];
      static int8_t stunde_memory=now.hour();
      char stunde[3];
      static int8_t minute_memory=now.minute();
      char minute[3];

      switch (hmi_input[2])
      {
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

  case 6: //Farbmix-page
      HMI_Input_loeschen(hmi_input);
  break;
  
  case 8:
    alarm_fired=false;
    if(rtc.alarmFired(1)){
      rtc.clearAlarm(1);
    }
    if(rtc.alarmFired(2)){
      rtc.clearAlarm(2);
    }
    Signalgeber(false);
    
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
      rtc.setAlarm1(DateTime(now.year(), now.month(), now.day(), alarm1_stunde_memory, alarm1_minute_memory, 0), DS3231_A1_Day); //Alarm when day (day of week), hours,inutes and seconds match */
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
      rtc.setAlarm2(DateTime(now.year(), now.month(), now.day(), alarm2_stunde_memory, alarm2_minute_memory, 0), DS3231_A2_Day); //Alarm when day (day of week), hours,inutes and seconds match */
    }else{
      rtc.clearAlarm(2);  //Alarm ausschalten
      noTone(4);  //Signalton ausschalten
    }


static uint8_t anzeige_zeit;
if(anzeige_zeit==100){
displayTime (false, now);
  if(rtc.alarmFired(1) && alarm1_ein_memory==true)
  {
    if(!alarm_fired){
      Serielle_Textausgabe2("page 8");
      alarm_fired=true;
    }
    Signalgeber(true);
  }
  if(rtc.alarmFired(2) && alarm2_ein_memory==true)
  {
    if(!alarm_fired){
      Serielle_Textausgabe2("page 8");
      alarm_fired=true;
    }
    Signalgeber(true);
  }
}
anzeige_zeit++;

  
}

void HMI_Input_loeschen(char* HMI_Input_array)
{
  for(int i=0; i<7;i++) //Inputdatenarray löschen
      {
        HMI_Input_array[i]="";
      }
}


void displayTime (bool uhr_einstellen, DateTime now) {
  //TODO: übergabeparameter kann entfernt werden, kann immer angezeigt werden in den feldern
  //DateTime now = rtc.now();
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

void setModusActive(){
  switch (modus){
    case 1: //Lernen
      red=255;
      green=255;
      blue=255;
      leuchtstaerke=255;
      break;
    case 2: //Entspannungslicht (Relax)
      red=241;
      green=142;
      blue=28;
      leuchtstaerke=255;
      break;
    case 4: //Party
      leuchtstaerke=255;
      break;
    case 3: //Farben mix
      break;
    case 6: //Lichtabhängige Lichtansteuerung (Automatik)
      break;
    default:
      break;
  }
}

int LDR_Messung()
{
  helligkeit = analogRead(0);
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

void Serielle_Textausgabe2(const char* textbox)
{
  for(int i=0;i<2;i++){
      Serial.print(textbox);
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
  return 1;
}

void Signalgeber(bool ton_an)
{
  if(ton_an==true){
    tone(4,264); // (pin, frequency, duration)
    delay(100);
    tone(4,464); // (pin, frequency, duration)
    delay(100);
  }else{
    noTone(4);
  }
}

uint8_t RGB_Licht_Funktion(uint8_t pixelnummer, uint8_t rot, uint8_t gruen, uint8_t blau, uint8_t helligkeit, uint8_t modi, bool hauptleuchte, bool indirekt)
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
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
      main_light.fill(main_light.Color(rot, gruen, blau));
      main_light.show();
    }
  } 
  return 1;
}

