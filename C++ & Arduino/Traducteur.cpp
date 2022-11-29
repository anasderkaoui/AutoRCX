// Translate dots/dashes to LED light pulses on the Arduino UNO board

#include "Traducteur.h" // Includes the class "Traducteur.h" and all its functions
#include "LettreMorse.h" // Includes the class "LettreMorse.h" and all its functions so that will not redefine it again

     
     
LettreMorse lettre=LettreMorse();

Traducteur::Traducteur(){}

     //int len;
     //char tab[]=" ";
     const int led = 13; // The corresponding pin to the LED of the Arduino UNO board
     int ledState = LOW;     // ledState used to set the LED
     //int period_dot = 100;
     //int period_dash = 1000;
     //unsigned long current_time = 0;
     // Generally, you should use "unsigned long" for variables that hold time
     // The value will quickly become too large for an int to store
     unsigned long currentMillis = millis();
     unsigned long previousMillis = 0;        // will store last time LED was updated

     // constants won't change:
     const long interval_dot = 100;           // interval at which to blink if it is a dot (milliseconds)
     const long interval_dash = 1000;           // interval at which to blink if it is a dash (milliseconds)

void Traducteur::dot()
{
if (currentMillis - previousMillis >= interval_dot) {  
    // Blink the LED if the difference between the current time and last time you blinked the LED is bigger than the interval at which you want to blink the LED.
    // save the last time you blinked the LED
     previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState==LOW) {
      digitalWrite(led, HIGH);;
    } else {
      digitalWrite(led, LOW);;
    }
}
}
void Traducteur::dash()
{
if (currentMillis - previousMillis >= interval_dash) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState==LOW) {
      digitalWrite(led, HIGH);;
    } else {
      digitalWrite(led, LOW);;
    }
}
}

// Morse code for : Dots/Dashes -> LED Blink

void Traducteur::morse(char ch[]){

for (int i = 0; i<sizeof(ch) ; i++){

if (ch == 'A' || ch == 'a')
{
lettre.convertisseur(ch);
dot();

dash();

Serial.print(" ");

}
else if (ch == 'B' || ch == 'b')
{
lettre.convertisseur(ch);
dash();

dot();

dot();

dot();

Serial.print(" ");

}
else if (ch == 'C' || ch == 'c')
{
lettre.convertisseur(ch);
dash();

dot();

dash();

dot();

Serial.print(" ");

}
else if (ch == 'D' || ch == 'd')
{
lettre.convertisseur(ch);
dash();

dot();

dot();

}
else if (ch == 'E' || ch == 'e')
{
lettre.convertisseur(ch);
dot();

}
else if (ch == 'F' || ch == 'f')
{
lettre.convertisseur(ch);
dot();

dot();

dash();

dot();

}
else if (ch == 'G' || ch == 'g')
{
lettre.convertisseur(ch);
dash();

dash();

dot();

}
else if (ch == 'H' || ch == 'h')
{
lettre.convertisseur(ch);
dot();

dot();

dot();

dot();

}
else if (ch == 'I' || ch == 'i')
{
lettre.convertisseur(ch);
dot();

dot();

}
else if (ch == 'J' || ch == 'j')
{
lettre.convertisseur(ch);
dot();

dash();

dash();

dash();

}
else if (ch == 'K' || ch == 'k')
{
lettre.convertisseur(ch);
dash();

dot();

dash();

}
else if (ch == 'L' || ch == 'l')
{
lettre.convertisseur(ch);
dot();

dash();

dot();

dot();

}
else if (ch == 'M' || ch == 'm')
{
lettre.convertisseur(ch);
dash();

dash();

}
else if (ch == 'N' || ch == 'n')
{
lettre.convertisseur(ch);
dash();

dot();

}
else if (ch == 'O' || ch == 'o')
{
lettre.convertisseur(ch);
dash();

dash();

dash();

}
else if (ch == 'P' || ch == 'p')
{
lettre.convertisseur(ch);
dot();

dash();

dash();

dot();
}
else if (ch == 'Q' || ch == 'q')
{
lettre.convertisseur(ch);
dash();

dash();

dot();

dash();

}
else if (ch == 'R' || ch == 'r')
{
lettre.convertisseur(ch);
dot();

dash();

dot();

}
else if (ch == 'S' || ch == 's')
{
lettre.convertisseur(ch);
dot();

dot();

dot();

}
else if (ch == 'T' || ch == 't')
{
lettre.convertisseur(ch);
dash();

}
else if (ch == 'U' || ch == 'u')
{
lettre.convertisseur(ch);
dot();

dot();

dash();

}
else if (ch == 'V' || ch == 'v')
{
lettre.convertisseur(ch);
dot();

dot();

dot();

dash();

}
else if (ch == 'W' || ch == 'w')
{
lettre.convertisseur(ch);
dot();

dash();

dash();

}
else if (ch == 'X' || ch == 'x')
{
lettre.convertisseur(ch);
dash();

dot();

dot();

dash();

}
else if (ch == 'Y' || ch == 'y')
{
lettre.convertisseur(ch);
dash();

dot();

dash();

dash();

}
else if (ch == 'Z' || ch == 'z')
{
lettre.convertisseur(ch);
dash();

dash();

dot();

dot();

}
else if (ch == '0')
{
lettre.convertisseur(ch);
dash();

dash();

dash();

dash();

dash();

}
else if (ch == '1')
{
lettre.convertisseur(ch);
dot();

dash();

dash();

dash();

dash();

}
else if (ch == '2')
{
lettre.convertisseur(ch);
dot();

dot();

dash();

dash();

dash();

}
else if (ch == '3')
{
lettre.convertisseur(ch);
dot();

dot();

dot();

dash();

dash();

}
else if (ch == '4')
{
lettre.convertisseur(ch);
dot();

dot();

dot();

dot();

dash();

}
else if (ch == '5')
{
lettre.convertisseur(ch);
dot();

dot();

dot();

dot();

dot();

}
else if (ch == '6')
{
lettre.convertisseur(ch);
dash();

dot();

dot();

dot();

dot();

}
else if (ch == '7')
{
lettre.convertisseur(ch);
dash();

dash();

dot();

dot();

dot();

}
else if (ch == '8')
{
lettre.convertisseur(ch);
dash();

dash();

dash();

dot();

dot();

}
else if (ch == '9')
{
lettre.convertisseur(ch);
dash();

dash();

dash();

dash();

dot();

}
else if(ch == ' ' || ch == '/')
{
dash();
Serial.print(" ");
}
else
{
Serial.println("Unknown symbol! Traducteur ");
}
}
}
