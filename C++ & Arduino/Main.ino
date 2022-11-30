// This is where the characters need to be typed

#include "Traducteur.h" // Includes "Traducteur.h" that has all the needed functions for the morse code to work (including "LettreMorse.h")

const int led = 13;  // The pin 13 that corresponds to the LED on the Arduino UNO board
Traducteur traducteur=Traducteur();  // Creats an object of type "Traducteur" to allow us to use the functions in "Traducteur.h" that are defined in "Traducteur.cpp"
char phrase[50] = "Put your phrase in here";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // The speed at which we can communicate with the Arduino UNO R3 board
  pinMode(led,OUTPUT);  // Defining pin that will be used as an output (13 our this case)
  Serial.print("Translation of ");
  Serial.println(phrase);
  traducteur.morse(phrase); // The input which will be translated to morse code
  Serial.println("Translation has finished, don't mind the errors that follow!"); // The errors are showing because the array "phrase" has empty columns in the end of it that have no defined value
}

void loop() {
  // put your main code here, to run repeatedly:
}
