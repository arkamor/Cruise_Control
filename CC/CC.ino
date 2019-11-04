#include <PID_v1.h>
#include <SPI.h>
#include <mcp2515.h>

#define PIN_BP_UP      2    // Commande au volant +
#define PIN_BP_DOWN    3    // Commande au volant -
#define PIN_ON_OFF     4    // Bouton ON/OFF
#define PIN_Pedale1    A0   // Pin analogique voie 1 pedale
#define PIN_Pedale2    A1   // Pin analogique voie 2 pedale
#define PIN_Rly_On_Off 5   // Bouton ON/OFF

struct can_frame canMsg;
MCP2515 mcp2515(10);

////////////////////////////////////////
// Declatation des variables globales //
////////////////////////////////////////

float const    P = 1.0;
float const    I = 1.0;
float const    D = 1.0;

double         Vitesse_Mesuree;
double         Vitesse_Delta;

double         Vitesse_Consigne;
double         Vitesse_Moteur;
double         Commande;

bool           Pedale_Frein;
bool           Rapport_Nok;
bool           Reg_ON;

unsigned long  last_BP_UP;
unsigned long  last_BP_DOWN;

PID myPID(&Vitesse_Mesuree, &Commande, &Vitesse_Consigne,P,I,D, DIRECT);

////////////////////
// Fonction setup //
////////////////////

void setup() {
  pinMode(PIN_BP_UP,INPUT);
  pinMode(PIN_BP_DOWN,INPUT);
  pinMode(PIN_ON_OFF,OUTPUT);
  //pinMode(PIN_Pedale,OUTPUT);
  //pinMode(PIN_Pedal_i,OUTPUT);
  
  Reg_ON = 0;
  Serial.begin(115200);
  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  attachInterrupt(digitalPinToInterrupt(PIN_BP_UP), PB_UP,   FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BP_DOWN), PB_DOWN, FALLING);
}

///////////////////
// Fonction loop //
///////////////////

void loop() {
  Receive();
  digitalWrite(PIN_ON_OFF, Reg_ON);
  if (Pedale_Frein || Rapport_Nok || (Vitesse_Consigne < 50)) {
    Reg_ON = 0;
  }
  else if (Reg_ON){
    myPID.Compute();
    //analogWrite(PIN_Pedale,Commande);
  }
}

////////////////////////////
// Fonction Rapport_Check //
////////////////////////////

void Rapport_Check(){                                                            // Fonction pour vérifier si la vitesse est 4eme ou 5eme et si la pedale d'embrayage n'est pas appuyée
  float rapport = Vitesse_Moteur/Vitesse_Mesuree;                                // Calcul du rapport actuel
  float rapport_4 = 27.64;                                                       // Rapport de la 4eme vitesse
  float rapport_5 = 23.92;                                                       // Rapport de la 5eme vitesse
  //    un peut moins du 4                  un peut plus du 4         \/     un peut moins du 5             un peut plus du 5
  if ( ((rapport > (rapport_4-0.5) ) && (rapport < (rapport_4+0.5) )) || ((rapport > (rapport_5-0.5) ) && (rapport < (rapport_5+0.5) ))) {   //dans la 4eme ou dans la 5eme
    Rapport_Nok = 0;
    return 0;
  }
  else {
    Rapport_Nok = 1;
    return 0;
  }
}

//////////////////////
// Fonction Receive //
//////////////////////

void Receive(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){                        //Message reçu sans erreur
    if (canMsg.can_id == 0xFA) {                                                 // Si l'ID est 0xFA (pour RPM et Frein)
      Vitesse_Moteur = ((canMsg.data[0]<<8) + canMsg.data[1])/8;                 // Mise en forme des octets pour la vitesse du moteur
      Pedale_Frein = (canMsg.data[4]&0x40)>>6;                                   // Bit appui sur pedale de frein
    }
    else if  (canMsg.can_id == 0x102) {                                          // Si l'ID est 0x102 (pour Vitesse)
      Vitesse_Mesuree = (float(canMsg.data[0]<<8) + canMsg.data[1])/96;          // Mise en forme des octets pour la vitesse de la voiture
      Serial.print("Rapport actuel = ");                                         ////// DEBUGG
      Serial.println(int(Vitesse_Moteur/Vitesse_Mesuree));                     ////// DEBUGG
    }
  }
}

void PB_UP(){
  if ((last_BP_UP+200) < millis()){
    if (Reg_ON){
      Vitesse_Consigne++;
      if (digitalRead(3)) Reg_ON = 0;
    }
    else {
      Vitesse_Consigne = Vitesse_Mesuree;
      Reg_ON = 1;
    }
    last_BP_UP = millis();
  }
}

void PB_DOWN(){
  if ((last_BP_DOWN+200) < millis()){
    if (Reg_ON){
      if (Vitesse_Consigne > 50) Vitesse_Consigne--;
      if (digitalRead(2)) Reg_ON = 0; 
    }
    else {
      Vitesse_Consigne = Vitesse_Mesuree;
      Reg_ON = 1;
    }
    last_BP_DOWN = millis();
  }
}
