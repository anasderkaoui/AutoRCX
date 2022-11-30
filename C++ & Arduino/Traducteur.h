// Contains the functions that will translate dashes and dots to LED light pulses on the Arduino UNO board accordingly

#include "LettreMorse.h" // Copies the content of the file "LettreMorse.h"

class Traducteur{
  
  public: // Contains the functions that take the input characters in the main code and translate them to light pulses
    
    Traducteur();
    void morse(char ch[]);  // Translates letters and numbers to LED blinking light with the aid of "LettreMorse.h"
    void dot();  // Translates dots to light pulses
    void dash();  // Translates dashes to light pulses
    
};
