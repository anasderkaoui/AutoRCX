// Translates dots and dashes to LED light pulses on the Arduino UNO R3 board

#include "Traducteur.h" // Includes the class "Traducteur.h" and all its functions
#include "LettreMorse.h" // Includes the class "LettreMorse.h" and all its functions so that will not redefine it again

     
     
LettreMorse lettre=LettreMorse();

Traducteur::Traducteur() {}

     const int led = 13; // The corresponding pin to the LED of the Arduino UNO board
     
     // Generally, you should use "unsigned long" for variables that hold time
     // The value will quickly become too large for an int to store
     
     unsigned long currentMillis = millis();
     unsigned long previousMillis = 0;        // will store last time LED was updated

     // constants won't change:
     const long interval_dot = 800;           // interval at which to blink if it is a dot (milliseconds)
     const long interval_dash = 1700;           // interval at which to blink if it is a dash (milliseconds)
     const long space = 500;  // the time the LED will transit from dot to dash or vice-versa

void Traducteur::dot()
{
  previousMillis = millis();  // is the varibale that has the time we got into the function

  digitalWrite(led, HIGH); // The LED will be turned ON
  while (currentMillis - previousMillis <= interval_dot) {  // This loop acts like a delay
    currentMillis = millis();  // is the variable that will have the time we exceeded the interval
  }
  digitalWrite(led, LOW);  // after the "delay loop alternative" has finished, we want the LED turned off
  previousMillis = millis();  // here we repeat the same thing as in the previous lines in this function, to replace a delay
  while (currentMillis - previousMillis <= space) {
    currentMillis = millis();
  }
  digitalWrite(led, LOW);

}
void Traducteur::dash()  // same for this function as for the one above
{
  previousMillis = millis();
  
  digitalWrite(led, HIGH);
  while (currentMillis - previousMillis <= interval_dash) {
    currentMillis = millis();
  }
  digitalWrite(led, LOW);
  previousMillis = millis();
  while (currentMillis - previousMillis <= space) {
    currentMillis = millis();
  }
  digitalWrite(led, LOW);

}

// Morse code for : Translating Dots and Dashes to LED Blink

void Traducteur::morse(char ch[100]) {  // The function takes a big array to store a lot of characters

  for (int i = 0; i< 100 ; i++){  // Here we will make sure to cover all the characters of the array.

    if (ch[i] == 'A' || ch[i] == 'a')
    {
      lettre.convertisseur(ch[i]);  // Sends the character "ch[i]" to "LettreMorse.cpp" and translates it to its morse code, then translates it in terms of LED blinking
      dot();  // The function that blinks the LED to match a dot

      dash();  // The function that blinks the LED to match a dash
      
    }
    else if (ch[i] == 'B' || ch[i] == 'b')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dot();

      dot();

    }
    else if (ch[i] == 'C' || ch[i] == 'c')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dash();

      dot();

    }
    else if (ch[i] == 'D' || ch[i] == 'd')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dot();

    }
    else if (ch[i] == 'E' || ch[i] == 'e')
    {
      if (ch[i] == 'E') {
        Serial.print(" . ");
        dot();
      }

      else {
        lettre.convertisseur(ch[i]);
        dot();

      }
    }
    else if (ch[i] == 'F' || ch[i] == 'f')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dash();

      dot();

    }
    else if (ch[i] == 'G' || ch[i] == 'g')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dot();

    }
    else if (ch[i] == 'H' || ch[i] == 'h')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dot();

      dot();

    }
    else if (ch[i] == 'I' || ch[i] == 'i')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

    }
    else if (ch[i] == 'J' || ch[i] == 'j')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dash();

      dash();

      dash();

    }
    else if (ch[i] == 'K' || ch[i] == 'k')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dash();

    }
    else if (ch[i] == 'L' || ch[i] == 'l')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dash();

      dot();

      dot();

    }
    else if (ch[i] == 'M' || ch[i] == 'm')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

    }
    else if (ch[i] == 'N' || ch[i] == 'n')
    {
      if (ch[i] == 'n') {
        Serial.print(" - . ");
        dash();
           
        dot();
      }

      else {
        lettre.convertisseur(ch[i]);
        dash();

        dot();

      }
    }
    else if (ch[i] == 'O' || ch[i] == 'o')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dash();

    }
    else if (ch[i] == 'P' || ch[i] == 'p')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dash();

      dash();

      dot();
    }
    else if (ch[i] == 'Q' || ch[i] == 'q')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dot();

      dash();

    }
    else if (ch[i] == 'R' || ch[i] == 'r')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dash();

      dot();

    }
    else if (ch[i] == 'S' || ch[i] == 's')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dot();

    }
    else if (ch[i] == 'T' || ch[i] == 't')
    {
      lettre.convertisseur(ch[i]);
      dash();

    }
    else if (ch[i] == 'U' || ch[i] == 'u')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dash();

    }
    else if (ch[i] == 'V' || ch[i] == 'v')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dot();

      dash();

    }
    else if (ch[i] == 'W' || ch[i] == 'w')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dash();

      dash();

    }
    else if (ch[i] == 'X' || ch[i] == 'x')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dot();

      dash();

    }
    else if (ch[i] == 'Y' || ch[i] == 'y')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dash();

      dash();

    }
    else if (ch[i] == 'Z' || ch[i] == 'z')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dot();

      dot();

    }
    else if (ch[i] == '0')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dash();

      dash();

      dash();

    }
    else if (ch[i] == '1')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dash();

      dash();

      dash();

      dash();

    }
    else if (ch[i] == '2')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dash();

      dash();

      dash();

    }
    else if (ch[i] == '3')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dot();

      dash();

      dash();

    }
    else if (ch[i] == '4')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dot();

      dot();

      dash();

    }
    else if (ch[i] == '5')
    {
      lettre.convertisseur(ch[i]);
      dot();

      dot();

      dot();

      dot();

      dot();

    }
    else if (ch[i] == '6')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dot();

      dot();

      dot();

      dot();

    }
    else if (ch[i] == '7')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dot();

      dot();

      dot();

    }
    else if (ch[i] == '8')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dash();

      dot();

      dot();

    }
    else if (ch[i] == '9')
    {
      lettre.convertisseur(ch[i]);
      dash();

      dash();

      dash();

      dash();

      dot();

    }
    else if (ch[i] == ' ' || ch[i] == '/')
    {
      dash();
      Serial.print(" ");
    }
    else
    {
      Serial.print("");
    }
  }
}
