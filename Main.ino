#include "Traducteur.h"

const int led = 13;
Traducteur traducteur=Traducteur();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  traducteur.morse('A8');
  pinMode(led,OUTPUT);
  Serial.println(millis());

}

void loop() {
  // put your main code here, to run repeatedly:
  //traducteur.delay_alternative_dot();
  //traducteur.delay_alternative_dash();
  //traducteur.morse('8');

}
