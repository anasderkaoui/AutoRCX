// This is where the characters need to be typed

#include "Traducteur.h" // Includes "Traducteur.h" that has all the needed functions for the morse code to work (including "LettreMorse.h")

const int led = 13;  // The pin 13 that corresponds to the LED on the Arduino UNO board
Traducteur traducteur=Traducteur();  // Creats an object of type "Traducteur" to allow us to use the functions in "Traducteur.h" that are defined in "Traducteur.cpp"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // The speed at which we can communicate with the Arduino UNO R3 board
  traducteur.morse('J');  // The input which will be translated to morse code
  pinMode(led,OUTPUT);  // Defining pin that will be used as an output (13 our this case)
  //Serial.println(millis());

}

void loop() {
  // put your main code here, to run repeatedly:
  // The two following functions help run the function "millis()" to count the time that the LED will be turned on/turned off
  traducteur.dot();
  traducteur.dash();
  //traducteur.morse('8');

}
