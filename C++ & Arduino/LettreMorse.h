// This is the class that will be converting letters to dashes and dots, the functions below will be defined in the file "LettreMorse.cpp"

#include <Arduino.h>
#ifndef LETTREMORSE_H // Enables us to use/copy the content of "LettreMorse.h" in all of the other files that we will need
#define LETTREMORSE_H

class LettreMorse{
  
  
  
public: // Functions that will be used in other files

  LettreMorse();
  void convertisseur(char lettre);

};

#endif
