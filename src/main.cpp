#include <Arduino.h>
//#include "avr8-stub.h"
//#include "app_api.h" //only needed with flash breakpoints
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include <avr/power.h>

#include <Wire.h> //wird gebraucht für I2C-Kommunikation mit dem Gestensensor
#include "RevEng_PAJ7620.h"

#include "Nextion.h"
#include "Config.h"




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

int helligkeit; //Wird benötigt für die LDR-Messung
bool flanke_Licht_ein = false;
int modus=0;  //1 Lernen, 2 Relax, 3 Mix, 4 Party, 6 Auto
int leuchtstaerke=0;
bool hauptleuchte_an=false;
bool indirektebeleuchtung_an=false;
int durchlaufzaehler_party_farbwechsel=0;
volatile int flankenzaehler_ein_aus=0;
int activeLamp=0; //0 beide aus; 1 Haupt; 2 Neben; 3 beide

int red,green,blue,bright;
uint32_t memory;


// Funktionen:
int RGB_Licht_Funktion(int, int, int, int, int, int, bool, bool); //(Pixel,R,G,B,Helligkeit,Modus,Hauptleuchte_an,Indirektebeleuchtung_an)
int Signalgeber(int, int); //(An, Modus)
int Gestensensor(); //Gestensensor
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


//Displayelemente
NexSlider h_red_ls = NexSlider(7, 1, "h_red_ls"); //Slider initialisieren rot; Touch-Release Event muss noch konfiguriert werden
NexSlider h_green_ls = NexSlider(7, 4, "h_green_ls"); //Slider gruen
NexSlider h_blue_ls = NexSlider(7, 6, "h_blue_ls"); //Slider blau
NexSlider h_hell_ls = NexSlider(7, 8, "h3");  //Slider helligkeit
NexDSButton bt_save_ls = NexDSButton(7, 2, "bt_save_ls"); //Button Licht an/aus
NexButton b_switch_ls = NexButton(7,15,"b_switch_ls");  //Button um die Lampen umzuschalten
NexDSButton bt_lernen_lk = NexDSButton(1,13,"bt_lernen_lk");
NexDSButton bt_relax_lk = NexDSButton(1,15,"bt_relax_lk");
NexDSButton bt_auto_lk = NexDSButton(1,14,"bt_auto_lk");
NexDSButton bt_party_lk = NexDSButton(1,16,"bt_party_lk");
NexDSButton bt_mix_lk = NexDSButton(1,3,"bt_mix_lk");
NexRadio r_hauptle_lk = NexRadio(1,10,"r_hauptle_lk");
NexRadio r_indirektb_lk = NexRadio(1,11,"r_indirektb_lk");

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
  &r_indirektb_lk
};

#pragma region DisplayFunctions
bool sendCmdToDisplay(String command){
  Serial.print(command);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void setColorsActive(){
  if (activeLamp==2)
  {
    main_light.fill(main_light.Color(red, green, blue));
    main_light.setBrightness(bright);
    main_light.show();
  }else /*if (activeLamp==3)*/{
    strip_IndLi.fill(strip_IndLi.Color(red, green, blue));
    strip_IndLi.setBrightness(bright);
    strip_IndLi.show();
    strip_IndRe.fill(strip_IndRe.Color(red, green, blue));
    strip_IndRe.setBrightness(bright);
    strip_IndRe.show();
  }
}

/*
void h0PushCallback(void *ptr)      fuer Touch Press Event*/

void h_red_lsPopCallback(void *ptr){
  h_red_ls.getValue(&memory);
  red=memory;
  setColorsActive();
}

void h_green_lsPopCallback(void *ptr){
  h_green_ls.getValue(&memory);
  green=memory;
  setColorsActive();
}

void h_blue_lsPopCallback(void *ptr){
  h_blue_ls.getValue(&memory);
  blue=memory;
  setColorsActive();
}

void h_hell_lsPopCallback(void *ptr){
  h_hell_ls.getValue(&memory);
  bright=memory;
  setColorsActive();
}

// TODO - Die Farbwerte für den aktuellen Modus speichern
void bt_save_lsPopCallback(void *ptr){
  uint32_t state=0;
  bt_save_ls.getValue(&state);

  if(state == 1){
    strip_IndLi.fill(strip_IndLi.Color(red, green, blue));
    strip_IndLi.setBrightness(bright);
    strip_IndLi.show();
  }else{
    strip_IndLi.setBrightness(0);
    strip_IndLi.show();
  }

}

void b_switch_lsPopCallback(void *ptr){
  String text;
  switch(activeLamp){
    case 0: activeLamp=1;
       text="Haupt-\rleuchte";
       break;
    case 1: activeLamp=2;
       text="Neben-\rleuchte";
       break;
    case 2: activeLamp=3;
       text="Beide\rLeuchten";
       break;
    default: activeLamp=0;
       text="Beide\raus";
       break;
  }
  Serielle_Textausgabe("b_switch_ls.txt=",text);
  setColorsActive();
}

void bt_lernen_lkPopCallback(void *ptr){
  modus=1; //Arbeitslicht (Lernen)
  Serielle_Textausgabe("t_actmod_lk.txt=","Lernen"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
}

void bt_relax_lkPopCallback(void *ptr){
  modus=2; //Entspannungslicht (Relax)
  Serielle_Textausgabe("t_actmod_lk.txt=","Relax"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
}

void bt_party_lkPopCallback(void *ptr){
  modus=4; //Party
  Serielle_Textausgabe("t_actmod_lk.txt=","Party"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
}

void bt_auto_lkPopCallback(void *ptr){
  modus=6; //Lichtabhängige Lichtansteuerung (Automatik)
  Serielle_Textausgabe("t_actmod_lk.txt=","Automatik"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
}

void bt_mix_lkPopCallback(void *ptr){
  modus=3; //Farben mix
  Serielle_Textausgabe("t_actmod_lk.txt=","Farbenmix"); //Ausgabetext in Textbox 1 auf der Seite Licht konfiguration (Seite2)
}

void r_hauptle_lkPopCallback(void *ptr){
  r_hauptle_lk.getValue(&memory);
  hauptleuchte_an=memory?true:false;
}

void r_indirektb_lkPopCallback(void *ptr){
  r_indirektb_lk.getValue(&memory);
  indirektebeleuchtung_an=memory?true:false;
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

  //HMI
  String cmd; 
  cmd+= "\"";
  for(int i=0 ; i<=2 ; i++) //Mithilfe dieser Schleife wird die Textbox 1 zurückgesetzt, muss immer 2 mal gemacht werden damit es zuverlässig funktioniert
  {
    Serial.print("vis b0,0"); //Hiding Button to next page TODO: Make page 1 available
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
  }
}


// Main-Code-Schleife, diese Methode wird ständig wiederholt
void loop() 
{
  nexLoop(nex_listen_list);

  // TODO: Je nach aktivem Modus die Lampen ansteuern?
  //RGB_Licht_Funktion(0, 0, 0, 0, 255, 4);
  //RGB_Licht_Funktion(0, 0, 0, 0, 255, 6);
  //Signalgeber(0,0);
  //Gestensensor();
  //RGB_Licht_Funktion(0, 0, 0, 0, 0, 6);

/*
  Serial.print("va0.val=42");   //Sending Code to the Display; this case: value of va0 is 42
  Serial.write(0xff);           //After every command three times this line
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("vis t3,0");   //Hiding object t3
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
*/

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
  Serial.println(helligkeit);
  delay(500);
  

/*
   // das Signal wird nur aller 5 Sekunden abgefragt
  delay(950);


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
 
  
*/
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
/*
int LDR_Messung()
{
  helligkeit = analogRead(0);  
  return helligkeit;
}
*/


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

