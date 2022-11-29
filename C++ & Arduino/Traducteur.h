// Contains the functions that will translate dashes and dots to LED light pulses on the Arduino UNO board accordingly

#include "LettreMorse.h" // Copies the content of the file "LettreMorse.h"
//#ifndef TRADUCTEUR_H
//#define TRADUCTEUR_H

class Traducteur{
  
  
  public: // Functions that take the input characters in the main code and translate them to light pulses
    
    Traducteur();
    void morse(char ch[]);  // Translates letters with the aid of "LettreMorse.h"
    //void delay_alternative_dot();
    //void delay_alternative_dash();
    void dot(); // Translates dots to light pulses
//     void dash();  // Translates dashes to light pulses
    
};

//#endif
