#include <Arduino.h>
//#include "avr8-stub.h"
//#include "app_api.h" //only needed with flash breakpoints
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
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

// TODO: Alle Displayelemente und die Funktionen extrahieren

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