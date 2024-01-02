#include <Arduino.h>
//#include "avr8-stub.h"
//#include "app_api.h" //only needed with flash breakpoints
#include <Adafruit_NeoPixel.h>
//#include <Adafruit_NeoMatrix.h>
#include <avr/power.h>

#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include "RevEng_PAJ7620.h"

#include "Nextion.h"

// Variablen:
#define LED_PIN_IndLi    6    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndLi 29    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define LED_PIN_IndRe    5    // LED Pin für die indirekte Beleuchtung auf der linken Seite an Pin 6
#define LED_COUNT_IndRe 14    //Anzahl einzelner Neopixel (RGB-LEDs) des LED-Streifens indirekte Beleuchtung auf der linken Seite
#define main_light_width 5
#define main_light_high 5
#define main_light_pin 3

#define PIN_LED 13
#define PIN_schalter 7

int helligkeit; //Wird benötigt für die LDR-Messung
bool flanke_Licht_ein = false;
int modus=0;  //1 Lernen, 2 Relax, 3 Mix, 4 Party, 6 Auto
int modNeu=0;
//int leuchtstaerke=0;
bool hauptleuchte_an=false;
bool indirektebeleuchtung_an=false;
int durchlaufzaehler_party_farbwechsel=0;
volatile int flankenzaehler_ein_aus=0;
int activeLamp=0; //0 beide aus; 1 Haupt; 2 Neben; 3 beide
uint8_t parRed[12]={128, 192, 239, 255, 239, 192, 128, 64, 17, 0, 17, 64};
uint8_t parBlue[13]={213, 159, 97, 43, 8, 1, 23, 69, 128, 187, 233, 255, 248};
uint8_t parGreen[14]={2, 5, 32, 78, 134, 189, 232, 254, 251, 224, 178, 122, 67, 24};
uint8_t parBright[11]={128, 169, 196, 202, 185, 149, 107, 71, 54, 60, 87};
int count=0;

int red,green,blue,bright;
//Speicher für den Mixmodus
int indRed,indGreen,indBlue,indBright;
int mainRed,mainGreen,mainBlue,mainBright;
uint32_t memory;


// Funktionen:
int RGB_Licht_Funktion(int, int, int, int, int, int, bool, bool); //(Pixel,R,G,B,Helligkeit,Modus,Hauptleuchte_an,Indirektebeleuchtung_an)
int Signalgeber(int, int); //(An, Modus)
int Gestensensor(); //Gestensensor
int LDR_Messung(); //LDR Messung zwischen 0 und 1023
void Serielle_Textausgabe(String, String); //Textausgabe zum HMI
void HMI_Input_loeschen(char*);
boolean setModusActive(int);
void partymodus();
void refreshColours();
void activateIndBel(int, int, int, int);
void activateHauptBel(int, int, int, int);
void cmdForDisplay(String); //Textausgabe zum HMI, nur einmal für bool Werte?
void sendValue(String, int);


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
Adafruit_NeoPixel main_light(25, main_light_pin, NEO_GRB + NEO_KHZ800);    // NeoPixel pixel object:
//Adafruit_NeoMatrix main_light(main_light_width, main_light_high,main_light_pin, NEO_MATRIX_TOP+NEO_MATRIX_LEFT+NEO_MATRIX_ROWS ,NEO_GRB+NEO_KHZ800);

//Gestensensorobjekt
RevEng_PAJ7620 sensor = RevEng_PAJ7620();

// TODO: Alle Displayelemente und die Funktionen extrahieren

//Displayelemente
NexSlider h_red_ls = NexSlider(7, 1, "h_red_ls"); //Slider initialisieren rot; Touch-Release Event muss noch konfiguriert werden
NexSlider h_green_ls = NexSlider(7, 4, "h_green_ls"); //Slider gruen
NexSlider h_blue_ls = NexSlider(7, 6, "h_blue_ls"); //Slider blau
NexSlider h_hell_ls = NexSlider(7, 8, "h3");  //Slider helligkeit
NexDSButton bt_save_ls = NexDSButton(7, 2, "bt_save_ls"); //Button Licht an/aus
NexButton b_switch_ls = NexButton(7,15,"b_switch_ls");  //Button um die Lampen umzuschalten
NexDSButton bt_lernen_lk = NexDSButton(1,9,"bt_lernen_lk");
NexDSButton bt_relax_lk = NexDSButton(1,11,"bt_relax_lk");
NexDSButton bt_auto_lk = NexDSButton(1,10,"bt_auto_lk");
NexDSButton bt_party_lk = NexDSButton(1,12,"bt_party_lk");
NexDSButton bt_mix_lk = NexDSButton(1,13,"bt_mix_lk");
NexRadio r_hauptle_lk = NexRadio(1,7,"r_hauptle_lk");
NexRadio r_indirektb_lk = NexRadio(1,8,"r_indirektb_lk");
NexButton b_mixco_lk = NexButton(1,14,"b_mixco_lk");

NexTouch *nex_listen_list[]={
  &h_red_ls,
  &h_green_ls,
  &h_blue_ls,
  &h_hell_ls,
  &bt_save_ls,
  &b_switch_ls,
  &bt_lernen_lk,
  &bt_relax_lk,
  &bt_auto_lk,
  &bt_party_lk,
  &bt_mix_lk,
  &r_hauptle_lk,
  &r_indirektb_lk,
  &b_mixco_lk
};

#pragma region DisplayFunctions
bool sendCmdToDisplay(String command){
  Serial.print(command);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void h_red_lsPopCallback(void *ptr){
  h_red_ls.getValue(&memory);
  red=memory;
  sendValue("n_red_ls.val=",red);
  refreshColours();
}

void h_green_lsPopCallback(void *ptr){
  h_green_ls.getValue(&memory);
  green=memory;
  sendValue("n_green_ls.val=",green);
  refreshColours();
}

void h_blue_lsPopCallback(void *ptr){
  h_blue_ls.getValue(&memory);
  blue=memory;
  sendValue("n_blue_ls.val=",blue);
  refreshColours();
}

void h_hell_lsPopCallback(void *ptr){
  h_hell_ls.getValue(&memory);
  bright=memory;
  sendValue("n_hell_ls.val=",bright);
  refreshColours();
}

//Die Farbwerte für den aktuellen Modus speichern
//Speichert die Werte für die aktiven Lampen
void bt_save_lsPopCallback(void *ptr){
  uint32_t state=0;
  bt_save_ls.getValue(&state);

  if(state == 1){
    if(activeLamp==1){
      indRed=red;
      indBlue=blue;
      indGreen=green;
      indBright=bright;
    }
    if(activeLamp==2){
      mainRed=red;
      mainBlue=blue;
      mainGreen=green;
      mainBright=bright;
    }
  }
  //cmdForDisplay("bt_save_ls.val=0");
}

void b_switch_lsPopCallback(void *ptr){
  String text;
  activeLamp++;
  switch(activeLamp){
    case 1: 
        hauptleuchte_an=true;
        indirektebeleuchtung_an=false;
        text="Haupt-\rleuchte";
        red=mainRed;
        blue=mainBlue;
        green=mainGreen;
        bright=mainBright;
        bt_save_ls.setValue(0);
        break;
    case 2: 
        hauptleuchte_an=false;
        indirektebeleuchtung_an=true;
        text="Neben-\rleuchte";
        red=indRed;
        blue=indBlue;
        green=indGreen;
        bright=indBright;
        bt_save_ls.setValue(0);
        break;
    default: 
        hauptleuchte_an=false;
        indirektebeleuchtung_an=false;
        text="Beide\raus";
        activeLamp=0;
        break;
  }
  Serielle_Textausgabe("b_switch_ls.txt=",text);
  refreshColours();
  delay(10);
  sendValue("n_hell_ls.val=",bright);
  delay(10);
  sendValue("n_red_ls.val=",red);
  delay(10);
  sendValue("n_blue_ls.val=",blue);
  delay(10);
  sendValue("n_green_ls.val=",green);
  delay(10);
  sendValue("h_hell_ls.val=",bright);
  delay(10);
  sendValue("h_red_ls.val=",red);
  delay(10);
  sendValue("h_blue_ls.val=",blue);
  delay(10);
  sendValue("h_green_ls.val=",green);
}

void bt_lernen_lkPopCallback(void *ptr){
  if(!setModusActive(1)){
    cmdForDisplay("bt_lernen_lk.val=1");
  }
}

void bt_relax_lkPopCallback(void *ptr){
  if(!setModusActive(2)){
    cmdForDisplay("bt_relax_lk.val=1");
  }
}

void bt_party_lkPopCallback(void *ptr){
  if(!setModusActive(4)){
    cmdForDisplay("bt_party_lk.val=1");
  }
}

void bt_auto_lkPopCallback(void *ptr){
  if(!setModusActive(6)){
    cmdForDisplay("bt_auto_lk.val=1");
  }
}

void bt_mix_lkPopCallback(void *ptr){
  if(!setModusActive(3)){
    cmdForDisplay("bt_mix_lk.val=1");
  }
}

//Konfigurationsbutton
void b_mixco_lkPopCallback(void *ptr){
  if(!setModusActive(3)){
    cmdForDisplay("bt_mix_lk.val=1");
  }
}

void r_hauptle_lkPopCallback(void *ptr){
  r_hauptle_lk.getValue(&memory);
  hauptleuchte_an=(memory==0)?false:true;
  refreshColours();
}

void r_indirektb_lkPopCallback(void *ptr){
  r_indirektb_lk.getValue(&memory);
  indirektebeleuchtung_an=(memory==0)?false:true;
  refreshColours();
}
#pragma endregion


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

  //Modi initialisieren
  //1 Lernen, 2 Relax, 3 Mix, 4 Party, 6 Auto


  //Display Events zu Artefakten hinzufuegen
  h_red_ls.attachPop(h_red_lsPopCallback);
  h_green_ls.attachPop(h_green_lsPopCallback);
  h_blue_ls.attachPop(h_blue_lsPopCallback);
  h_hell_ls.attachPop(h_hell_lsPopCallback);
  bt_save_ls.attachPop(bt_save_lsPopCallback);
  b_switch_ls.attachPop(b_switch_lsPopCallback);
  bt_lernen_lk.attachPop(bt_lernen_lkPopCallback);
  bt_relax_lk.attachPop(bt_relax_lkPopCallback);
  bt_auto_lk.attachPop(bt_auto_lkPopCallback);
  bt_party_lk.attachPop(bt_party_lkPopCallback);
  bt_mix_lk.attachPop(bt_mix_lkPopCallback);
  r_hauptle_lk.attachPop(r_hauptle_lkPopCallback);
  r_indirektb_lk.attachPop(r_indirektb_lkPopCallback);
  b_mixco_lk.attachPop(b_mixco_lkPopCallback);
}


// Main-Code-Schleife, diese Methode wird ständig wiederholt
void loop() 
{
  nexLoop(nex_listen_list);
  partymodus();

  /*
  if(modus==4){
    partymodus();
  }
  */
  //RGB_Licht_Funktion(0, 0, 0, 0, 255, 4);
  //RGB_Licht_Funktion(0, 0, 0, 0, 255, 6);
  //Signalgeber(0,0);
  //Gestensensor();
  //RGB_Licht_Funktion(0, 0, 0, 0, 0, 6);
}

boolean setModusActive(int newMod){
  if(modus==newMod){
    return 0;
  }
  //alten Modus abschalten
  switch(modus){
    case 1:
      cmdForDisplay("bt_lernen_lk.val=0");
      //Serielle_Textausgabe("bt_lernen_lk.val=","0"); 
      break;
    case 2:
      cmdForDisplay("bt_relax_lk.val=0");
      break;
    case 3:
      cmdForDisplay("bt_mix_lk.val=0");
      break;
    case 4:
      cmdForDisplay("bt_party_lk.val=0");
      break;
    case 6:
      cmdForDisplay("bt_auto_lk.val=0");
      break;
  }

  switch (newMod){
    case 1: //Lernen
      red=255;
      green=255;
      blue=255;
      bright=255;
      Serielle_Textausgabe("t_actmod_lk.txt=","Lernen"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      break;
    case 2: //Entspannungslicht (Relax)
      red=241;
      green=142;
      blue=28;
      bright=255;
      Serielle_Textausgabe("t_actmod_lk.txt=","Relax"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      break;
    case 4: //Party
      Serielle_Textausgabe("t_actmod_lk.txt=","Party"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
        //wird in anderen Funktion gehändelt
        //Task aktivieren?
      break;
    case 3: //Farben mix
      Serielle_Textausgabe("t_actmod_lk.txt=","Farbenmix"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      activateHauptBel(mainRed,mainGreen,mainBlue,mainBright);
      activateIndBel(indRed, indGreen, indBlue, indBright);
      break;
    case 6: //Lichtabhängige Lichtansteuerung (Automatik)
      red=255;
      green=0;
      blue=0;
      bright=150;
      Serielle_Textausgabe("t_actmod_lk.txt=","Automatik"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
      refreshColours();
      break;
    default:
      break;
  }
  modus=newMod;
  return 1;
}

void refreshColours(){
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
}

void activateIndBel(int rot, int gruen, int blau, int hell){
  strip_IndLi.fill(strip_IndLi.Color(rot, gruen, blau));
  strip_IndLi.setBrightness(hell);
  strip_IndLi.show();
  strip_IndRe.fill(strip_IndRe.Color(rot, gruen, blau));
  strip_IndRe.setBrightness(hell);
  strip_IndRe.show();
}

void activateHauptBel(int rot, int gruen, int blau, int hell){
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
  delay(50);
}

int LDR_Messung()
{
  helligkeit = analogRead(0);
  Serial.println(helligkeit);
  delay(500);
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

void cmdForDisplay(String cmd)
{
  Serial.print(cmd);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void sendValue(String cmd, int value)
{
  char hex[4];

  sprintf(hex, "%d", value);
  Serielle_Textausgabe(cmd,hex);

  //cmdForDisplay(cmd+hex);
  //cmdForDisplay(cmd+hex);
}

/*So wird es in der Nextion Bib gemacht
void sendCommand(const char* cmd)
{
    while (nexSerial.available())
    {
        nexSerial.read();
    }
    
    nexSerial.print(cmd);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
}*/

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

/*
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
*/
