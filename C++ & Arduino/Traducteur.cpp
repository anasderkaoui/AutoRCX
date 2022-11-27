

#include "Traducteur.h"
#include "LettreMorse.h"

     
     
LettreMorse lettre=LettreMorse();

Traducteur::Traducteur(){}

     //int len;
     //Traducteur traducteur=Traducteur();
     //char tab[]=" ";
     const int led = 13;
     // Variables will change:
     int ledState = LOW;             // ledState used to set the LED
     //int period_dot = 100;
     //int period_dash = 1000;
     //unsigned long current_time = 0;
     unsigned long currentMillis = millis();
     unsigned long previousMillis = 0;        // will store last time LED was updated

     // constants won't change:
     const long interval_dot = 100;           // interval at which to blink (milliseconds)
     const long interval_dash = 1000;           // interval at which to blink (milliseconds)

void delay_alternative_dot()
{
//while(millis() <= current_time + period_dot){
//Serial.println("jj");
if (currentMillis - previousMillis >= interval_dot) {
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
void delay_alternative_dash()
{
//if(millis() >= current_time + period_dash){
//Serial.println("jj");

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

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store

/*void loop() {
  // here is where you'd put code that needs to be running all the time.

  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
*/

void dot()
{
//Serial.print(".");
//digitalWrite(led, HIGH);
delay_alternative_dot();
//digitalWrite(led, LOW);
delay_alternative_dash();
}
void dash()
{
//Serial.print("-");
//digitalWrite(led, HIGH);
delay_alternative_dash();
//digitalWrite(led, LOW);
delay_alternative_dash();
}
// Morse

void Traducteur::morse(char ch){


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
delay_alternative_dash();
Serial.print(" ");
}
else
{
Serial.println("Unknown symbol! Traducteur ");
}
}
